package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class Motor extends SubsystemBase {
  private SparkMax motor;
  private RelativeEncoder encoder;

  public Motor() {

    motor = new SparkMax(Constants.CAN.MOTOR, MotorType.kBrushless);
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.MOTOR.CURRENT_LIMIT)
        .inverted(false);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    encoder = motor.getEncoder();
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor/Output", motor.getAppliedOutput());
    SmartDashboard.putNumber("Motor/Velocity RPM", encoder.getVelocity());
    SmartDashboard.putNumber("Motor/Current Amps", motor.getOutputCurrent());
    SmartDashboard.putNumber("Motor/Temperature C", motor.getMotorTemperature());
  }
}
