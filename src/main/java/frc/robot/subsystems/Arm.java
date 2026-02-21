// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase {
  private double angle; // Current height of the arm in meters
  private PIDController pidController; // PID controller for arm height control
  private ArmFeedforward feedforward; // Feedforward for arm control
  private SparkMax motor;
  private SparkAbsoluteEncoder encoder;
  /** Creates a new Arm. */
  public Arm() {
    this.feedforward = new ArmFeedforward(
      Constants.IntakeConstants.ArmConstants.kS,
      Constants.IntakeConstants.ArmConstants.kG,
      Constants.IntakeConstants.ArmConstants.kV,
      Constants.IntakeConstants.ArmConstants.kA
    );
    angle = 0.0; // Initialize the arm angle to 0 degrees
    this.pidController = new PIDController(
      Constants.IntakeConstants.ArmConstants.kP,
      Constants.IntakeConstants.ArmConstants.kI,
      Constants.IntakeConstants.ArmConstants.kD
    );
    this.motor = new SparkMax(Constants.IntakeConstants.ArmConstants.CAN, SparkLowLevel.MotorType.kBrushless);
    this.encoder = motor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    angle = encoder.getPosition();
    SmartDashboard.putNumber("Elevator/ArmEncoderPosition", angle);
    SmartDashboard.putNumber("Elevator/ArmEncoderVelocity", encoder.getVelocity());
  }

  public void setAngle(double targetAngle) {
    double pid = pidController.calculate(angle, targetAngle);
    double ff = feedforward.calculate(targetAngle, 0);
    motor.setVoltage(pid + ff);
  }

  public void stop() {
    motor.stopMotor();
  }
}
