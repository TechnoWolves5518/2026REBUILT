// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LauncherConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LauncherRotate extends SubsystemBase {
  private double angle; // Current height of the arm in meters
  private PIDController pidController; // PID controller for arm height control
  private ArmFeedforward feedforward; // Feedforward for arm control
  private SparkMax motor;
  private SparkAbsoluteEncoder encoder;
  /** Creates a new LauncherRotate. */
  public LauncherRotate() {
    this.feedforward = new ArmFeedforward(
      LauncherConstants.RotatorConstants.kS,
      LauncherConstants.RotatorConstants.kG,
      LauncherConstants.RotatorConstants.kV,
      LauncherConstants.RotatorConstants.kA
    );
    angle = 0.0; // Initialize the arm angle to 0 degrees
    this.pidController = new PIDController(
      LauncherConstants.RotatorConstants.kP,
      LauncherConstants.RotatorConstants.kI,
      LauncherConstants.RotatorConstants.kD
    );
    this.motor = new SparkMax(LauncherConstants.RotatorConstants.CAN, SparkLowLevel.MotorType.kBrushless);
    this.encoder = motor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    angle = encoder.getPosition() * LauncherConstants.RotatorConstants.gearRatio;
    SmartDashboard.putNumber("Launcher/ArmEncoderPosition", angle);
    SmartDashboard.putNumber("Launcher/ArmEncoderVelocity", encoder.getVelocity());
  }

  public void setAngle(double targetAngle) {
    double pid = pidController.calculate(angle, targetAngle);
    double ff = feedforward.calculate(targetAngle, 0);
    motor.setVoltage(pid + ff);
  }

  public void setVoltage(double targetVoltage) {
    motor.setVoltage(targetVoltage);
  }

  public void stop() {
    motor.stopMotor();
  }
  
  public Command runAngle(double Angle) {
    return this.run(() -> setAngle(Angle));
  }

  public Command runVoltage(double Voltage) {
    return this.run(() -> setVoltage(Voltage));
  }

  public Command stopCommand() {
    return this.run(this::stop);
  }
}
