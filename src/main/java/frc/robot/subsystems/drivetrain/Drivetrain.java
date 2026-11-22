package frc.robot.subsystems.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AUTON;
import frc.robot.Constants.CAN;
import frc.robot.Constants.SWERVE;
import frc.robot.Constants.VISION;
import frc.robot.sensors.AprilTagCamera;
import frc.robot.util.KnownLocations;

public class Drivetrain extends SubsystemBase {

  private MAXSwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
  private Canandgyro gyro;
  private SwerveDrivePoseEstimator odometry;
  private SwerveDriveKinematics swerveKinematics;
  private AprilTagCamera aprilTagCam;
  private Field2d smartdashField;

  private boolean isAprilTagCamUpdating = false;
  private int visionTagsSeen = 0;
  private double visionAgeSeconds = -1;

  public Drivetrain() {
    frontLeftModule = new MAXSwerveModule(CAN.DRIVING_FRONT_LEFT, CAN.TURNING_FRONT_LEFT, -Math.PI / 2);
    frontRightModule = new MAXSwerveModule(CAN.DRIVING_FRONT_RIGHT, CAN.TURNING_FRONT_RIGHT, 0);
    backLeftModule = new MAXSwerveModule(CAN.DRIVING_BACK_LEFT, CAN.TURNING_BACK_LEFT, Math.PI);
    backRightModule = new MAXSwerveModule(CAN.DRIVING_BACK_RIGHT, CAN.TURNING_BACK_RIGHT, Math.PI / 2);

    gyro = new Canandgyro(CAN.GYRO_DRIVETRAIN);
    CanandgyroSettings gyroSettings = new CanandgyroSettings();
    gyro.setSettings(gyroSettings);

    aprilTagCam = new AprilTagCamera(VISION.APRILTAG_CAM_NAME, VISION.APRILTAG_CAM_TRANSFORM);

    smartdashField = new Field2d();
    SmartDashboard.putData("Swerve Odometry", smartdashField);

    double widthFromCenter = Units.inchesToMeters(SWERVE.WHEEL_WIDTH) / 2;
    double lengthFromCenter = Units.inchesToMeters(SWERVE.WHEEL_LENGTH) / 2;

    swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(lengthFromCenter, widthFromCenter),
        new Translation2d(lengthFromCenter, -widthFromCenter),
        new Translation2d(-lengthFromCenter, widthFromCenter),
        new Translation2d(-lengthFromCenter, -widthFromCenter));

    odometry = new SwerveDrivePoseEstimator(
        swerveKinematics,
        getRotation(),
        getModulePositions(),
        new Pose2d(KnownLocations.getFieldLayout().getFieldLength() / 2,
            KnownLocations.getFieldLayout().getFieldWidth() / 2, getRotation()));

    configureAutoBuilder();
  }

  private void configureAutoBuilder() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          () -> getPose(),
          (pose) -> resetPose(pose),
          () -> getRobotRelativeSpeeds(),
          (speeds, feedforwards) -> drive(speeds),
          new PPHolonomicDriveController(AUTON.TRANSLATION_PID, AUTON.ROTATION_PID),
          config,
          () -> KnownLocations.getKnownLocations().alliance == Alliance.Red,
          this);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

    PathPlannerLogging.setLogActivePathCallback((poses) -> smartdashField.getObject("path").setPoses(poses));
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
    };
  }

  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    backLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    backRightModule.resetEncoders();
  }

  public void resetGyro() {
    gyro.setYaw(0);
    resetPose(new Pose2d(getPose().getTranslation(), KnownLocations.getKnownLocations().zeroGyroRotation));
  }

  public void recalibrateGyro() {
    gyro.startCalibration();
  }

  public void drive(double xSpeed, double ySpeed, double angularSpeed) {
    drive(xSpeed, ySpeed, angularSpeed, true);
  }

  public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldRelative) {
    drive(xSpeed, ySpeed, angularSpeed, fieldRelative,
        () -> this.getPose().getRotation().plus(KnownLocations.getKnownLocations().zeroGyroRotation));
  }

  public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldRelative,
      Supplier<Rotation2d> rotationSupplier) {
    double xSpeedDelivered = xSpeed * SWERVE.MAX_DIRECTION_SPEED;
    double ySpeedDelivered = ySpeed * SWERVE.MAX_DIRECTION_SPEED;
    double angularSpeedDelivered = angularSpeed * SWERVE.MAX_ROTATIONAL_SPEED;

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, angularSpeedDelivered);

    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotationSupplier.get());
    }

    drive(speeds);
  }

  public void drive(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, TimedRobot.kDefaultPeriod);
    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(discretizedSpeeds);

    setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SWERVE.MAX_DIRECTION_SPEED);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  public void lockSwerve() {
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(-Math.PI / 4)));
    backLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(-Math.PI / 4)));
    backRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(Math.PI / 4)));
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(getRotation(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveKinematics.toChassisSpeeds(new SwerveModuleState[] {
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState()
    });
  }

  public Rotation2d getRotation() {
    return gyro.getRotation2d().plus(Rotation2d.fromDegrees(180));
  }

  private boolean addVisionMeasurement(AprilTagCamera cam) {
    Optional<EstimatedRobotPose> result = cam.getGlobalPose();
    if (result.isEmpty()) {
      return false;
    }

    visionTagsSeen = result.get().targetsUsed.size();
    visionAgeSeconds = Timer.getFPGATimestamp() - result.get().timestampSeconds;
    smartdashField.getObject("Vision Pose").setPose(result.get().estimatedPose.toPose2d());

    if (!cam.rejectUpdate(result.get())) {
      odometry.addVisionMeasurement(
          result.get().estimatedPose.toPose2d(),
          result.get().timestampSeconds,
          cam.getEstimationStdDevs());
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    odometry.update(getRotation(), getModulePositions());

    isAprilTagCamUpdating = addVisionMeasurement(aprilTagCam);

    smartdashField.setRobotPose(getPose());

    SmartDashboard.putNumber("Drivetrain/CurrentPose X", getPose().getX());
    SmartDashboard.putNumber("Drivetrain/CurrentPose Y", getPose().getY());
    SmartDashboard.putNumber("Drivetrain/CurrentPose Rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Drivetrain/getRotation", getRotation().getDegrees());
    SmartDashboard.putBoolean("Drivetrain/isAprilTagCamUpdating", isAprilTagCamUpdating);
    SmartDashboard.putBoolean("Drivetrain/isAprilTagCamConnected", aprilTagCam.isConnected());
    SmartDashboard.putNumber("Drivetrain/Vision Tags Seen", visionTagsSeen);
    SmartDashboard.putNumber("Drivetrain/Vision Age Seconds", visionAgeSeconds);
    SmartDashboard.putNumber("Drivetrain/FL Drive Meters", frontLeftModule.getPosition().distanceMeters);
    SmartDashboard.putNumber("Drivetrain/FR Drive Meters", frontRightModule.getPosition().distanceMeters);
    SmartDashboard.putNumber("Drivetrain/BL Drive Meters", backLeftModule.getPosition().distanceMeters);
    SmartDashboard.putNumber("Drivetrain/BR Drive Meters", backRightModule.getPosition().distanceMeters);
  }
}
