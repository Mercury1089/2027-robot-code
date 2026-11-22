package frc.robot.sensors;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.KnownLocations;

public class AprilTagCamera extends PhotonCamera {

    private PhotonPoseEstimator estimator;

    private Matrix<N3, N1> curStdDevs;
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    private static final double MIN_TAG_AREA = 0.15;
    private static final double MAX_AMBIGUITY = 0.2;
    private static final double MAX_SINGLE_TAG_DISTANCE = 4.0;

    public AprilTagCamera(String name, Transform3d robotToCam) {
        super(name);
        estimator = new PhotonPoseEstimator(KnownLocations.getFieldLayout(), robotToCam);
        curStdDevs = kSingleTagStdDevs;
    }

    public Optional<EstimatedRobotPose> getGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult result : getAllUnreadResults()) {
            visionEst = estimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = estimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());
        }
        return visionEst;
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            curStdDevs = kSingleTagStdDevs;
            return;
        }

        Matrix<N3, N1> estStdDevs = kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        for (PhotonTrackedTarget tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            curStdDevs = kSingleTagStdDevs;
        } else {
            avgDist /= numTags;
            if (numTags > 1) estStdDevs = kMultiTagStdDevs;
            if (numTags == 1 && avgDist > MAX_SINGLE_TAG_DISTANCE) {
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            }
            curStdDevs = estStdDevs;
        }
    }

    public boolean rejectUpdate(EstimatedRobotPose estimatedPose) {
        List<PhotonTrackedTarget> usedTags = estimatedPose.targetsUsed;

        double largestArea = 0.0;

        for (PhotonTrackedTarget t : usedTags) {
            if (t.getArea() > largestArea) {
                largestArea = t.getArea();
            }
        }

        if (largestArea < MIN_TAG_AREA) {
            return true;
        }

        if (usedTags.size() == 1 && usedTags.get(0).getPoseAmbiguity() > MAX_AMBIGUITY) {
            return true;
        }

        return false;
    }
}
