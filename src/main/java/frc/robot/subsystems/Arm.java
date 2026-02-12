// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private double angle; // Current height of the arm in meters
  private PIDController pidController; // PID controller for arm height control
  /** Creates a new Arm. */
  public Arm(PIDController pidController) {
    this.pidController = pidController;
    angle = 0.0; // Initialize the arm angle to 0 degrees
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAngle(double targetAngle) {
    // Convert target angle to height using the arm's geometry
    double targetHeight = Math.sin(Math.toRadians(targetAngle)) * 1.0; // Assuming arm length is 1 meter
    
  }
}
