package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.detection.PhotonRobotDetector;
import com.pathplanner.lib.obstacle.ObstacleSettings;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DETECTION;
import frc.robot.Constants.DS_USB;
import frc.robot.Constants.JOYSTICK_BUTTONS;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Motor;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class RobotContainer {

  private CommandJoystick rightJoystick, leftJoystick;
  private CommandXboxController gamepad;

  private Trigger left10, left11;

  private Supplier<Double> gamepadLeftY;
  private Supplier<Double> driveX, driveY, driveRotation;

  private Drivetrain drivetrain;
  private Motor motor;
  private SendableChooser<Command> autoChooser;
  private PhotonRobotDetector robotDetector;

  private boolean cubedDrive = true;

  public RobotContainer() {
    leftJoystick = new CommandJoystick(DS_USB.LEFT_STICK);
    rightJoystick = new CommandJoystick(DS_USB.RIGHT_STICK);
    gamepad = new CommandXboxController(DS_USB.GAMEPAD);
    configureBindings();

    drivetrain = new Drivetrain();
    drivetrain.resetGyro();

    motor = new Motor();

    configureDetection();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    SmartDashboard.putBoolean("Drivetrain/Cubed Drive", cubedDrive);

    drivetrain.setDefaultCommand(DriveCommands.joyStickDrive(driveX, driveY, driveRotation, () -> cubedDrive, drivetrain));

    motor.setDefaultCommand(new RunCommand(
      () -> motor.setSpeed(-MathUtil.applyDeadband(gamepadLeftY.get(), Constants.MOTOR.JOYSTICK_DEADBAND)), motor));

    left10.onTrue(new InstantCommand(() -> drivetrain.resetGyro(), drivetrain).ignoringDisable(true));
    left11.onTrue(new InstantCommand(() -> drivetrain.recalibrateGyro(), drivetrain).ignoringDisable(true));
  }

  private void configureDetection() {
    robotDetector = new PhotonRobotDetector(
        DETECTION.CAM_NAME, DETECTION.CAM_TRANSFORM, () -> drivetrain.getPose(), DETECTION.ROBOT_CLASS_IDS);

    ObstacleSettings.configure(robotDetector, () -> drivetrain.getPose(), () -> drivetrain.drive(new ChassisSpeeds()));
  }

  private void configureBindings() {
    left10 = leftJoystick.button(JOYSTICK_BUTTONS.BTN10);
    left11 = leftJoystick.button(JOYSTICK_BUTTONS.BTN11);

    gamepadLeftY = () -> gamepad.getLeftY();

    driveX = () -> leftJoystick.getY();
    driveY = () -> leftJoystick.getX();
    driveRotation = () -> rightJoystick.getX();
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public PhotonRobotDetector getRobotDetector() {
    return robotDetector;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
