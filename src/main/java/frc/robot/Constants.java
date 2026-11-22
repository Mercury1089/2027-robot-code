package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static class DS_USB {
    public static final int LEFT_STICK = 0, RIGHT_STICK = 1, GAMEPAD = 2;

    private DS_USB() {
    }
  }

  public static class JOYSTICK_BUTTONS {
    public static final int BTN10 = 10;
    public static final int BTN11 = 11;

    private JOYSTICK_BUTTONS() {
    }
  }

  public static class SWERVE {
    public static final double WHEEL_WIDTH = 27 - 2 * (2 + 5.0 / 8);
    public static final double WHEEL_LENGTH = 32 - 2 * (2 + 5.0 / 8);

    public static final double WHEEL_DIAMETER = 0.0762; // meters
    public static final int PINION_TEETH = 13;

    public static final double MOTOR_REDUCTION = (45.0 * 22) / (PINION_TEETH * 15);

    public static final double DRIVING_MOTOR_FREE_SPEED = NEO_MOTOR_CONSTANTS.NEO_FREE_SPEED_RPMS / 60; // rps
    public static final double DRIVE_WHEEL_FREE_SPEED =
    (DRIVING_MOTOR_FREE_SPEED * (WHEEL_DIAMETER * Math.PI)) / MOTOR_REDUCTION;

    public static final double MAX_DIRECTION_SPEED = 4; // meters per second
    public static final double MAX_ROTATIONAL_SPEED = 2 * Math.PI; // radians per second

    public static final double JOYSTICK_DEADBAND = 0.15;

    private SWERVE() {
    }
  }

  public static final class NEO_MOTOR_CONSTANTS {
    public static final double NEO_FREE_SPEED_RPMS = 5676.0;

    private NEO_MOTOR_CONSTANTS() {
    }
  }

  public static class CAN {
    public static final int DRIVING_FRONT_LEFT = 3;
    public static final int TURNING_FRONT_LEFT = 4;
    public static final int DRIVING_FRONT_RIGHT = 7;
    public static final int TURNING_FRONT_RIGHT = 8;
    public static final int DRIVING_BACK_LEFT = 1;
    public static final int TURNING_BACK_LEFT = 2;
    public static final int DRIVING_BACK_RIGHT = 5;
    public static final int TURNING_BACK_RIGHT = 6;

    public static final int GYRO_DRIVETRAIN = 9;

    public static final int MOTOR = 11;

    private CAN() {
    }
  }

  public static class DETECTION {
    public static final String CAM_NAME = "DetectionCam";

    public static final Transform3d CAM_TRANSFORM = new Transform3d(
        new Translation3d(Units.inchesToMeters(16.0), Units.inchesToMeters(0.0), Units.inchesToMeters(5.0)),
        new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0)));

    public static final int[] ROBOT_CLASS_IDS = {1};

    private DETECTION() {
    }
  }

  public static class AUTON {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants ROTATION_PID = new PIDConstants(5.0, 0.0, 0.0);

    private AUTON() {
    }
  }

  public static class MOTOR {
    public static final int CURRENT_LIMIT = 40; // amps
    public static final double JOYSTICK_DEADBAND = 0.1;

    private MOTOR() {
    }
  }

  public static class VISION {
    public static final String APRILTAG_CAM_NAME = "ApriltagCam";

    public static final Transform3d APRILTAG_CAM_TRANSFORM = new Transform3d(
        new Translation3d(Units.inchesToMeters(-16.0), Units.inchesToMeters(-2.0), Units.inchesToMeters(5.0)),
        new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0)));

    private VISION() {

    }
  }
}
