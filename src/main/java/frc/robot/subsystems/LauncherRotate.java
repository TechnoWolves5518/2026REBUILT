// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;


import frc.robot.Constants.LauncherConstants.RotatorConstants;



public class LauncherRotate extends SubsystemBase {
  private PIDController pidController; // PID controller for arm height control
  private ArmFeedforward feedforward; // Feedforward for arm control
  private final SparkMax m_motor;
  private SparkAbsoluteEncoder encoder;
  /** Creates a new LauncherRotate. */
  public LauncherRotate() {
    m_motor = new SparkMax(RotatorConstants.CAN, SparkLowLevel.MotorType.kBrushless);
    m_motor.setInverted(true);
    encoder = m_motor.getAbsoluteEncoder();
    this.feedforward = new ArmFeedforward(
      RotatorConstants.kS,
      RotatorConstants.kG,
      RotatorConstants.kV,
      RotatorConstants.kA);
    this.pidController = new PIDController(
      RotatorConstants.kP,
      RotatorConstants.kI,
      RotatorConstants.kD);
  }

  public void setVoltage(double targetVoltage) {
    m_motor.setVoltage(targetVoltage);
  }

  public void stop() {
    m_motor.stopMotor();
  }
  
  public Command runVoltage(double Voltage) {
    return this.run(() -> setVoltage(Voltage));
  }
  public Command stopCommand() {
    return this.run(this::stop);
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
