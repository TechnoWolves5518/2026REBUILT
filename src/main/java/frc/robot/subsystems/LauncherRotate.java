// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LauncherConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;

import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Subsystem for controlling the launcher's rotational arm.
 * Uses a SparkMax motor with a relative encoder, SimpleMotorFeedforward, and
 * ProfiledPIDController to achieve precise angles for aiming the launcher.
 */
public class LauncherRotate extends SubsystemBase {
  @AutoLogOutput(key="Launcher/Arm/EncoderPosition")
  private double angle; // Current angle of the arm in degrees
  @AutoLogOutput(key="Launcher/Arm/Setpoint")
  private double setpoint;
  private ProfiledPIDController pidController; // PID controller for arm height control
  private SimpleMotorFeedforward feedforward; // Feedforward for arm control
  private SparkMax motor;
  @SuppressWarnings("unused")
  private SparkMaxSim motorSim;
  private RelativeEncoder encoder;
  private double goodDistance = LauncherConstants.RotatorConstants.launcherKnownGoodDistance;
  private double goodAngle = LauncherConstants.RotatorConstants.launcherKnownGoodAngle;
  private double slope = LauncherConstants.RotatorConstants.launcherAngleSlope;
  private boolean systemRun = true;
  @AutoLogOutput(key="Launcher/Arm/EncoderVelocity")
  private double currentVelocity;
  @AutoLogOutput(key="Launcher/Arm/AppliedVoltage")
  private double appliedVoltage;
  @AutoLogOutput(key="Launcher/Arm/AppliedCurrent")
  private double appliedCurrent;


  /** Creates a new LauncherRotate subsystem. */
  public LauncherRotate() {

    this.setpoint = LauncherConstants.RotatorConstants.defaultSetpoint;
    
    // Initialize feedforward
    this.feedforward = new SimpleMotorFeedforward(
      LauncherConstants.RotatorConstants.kS,
      LauncherConstants.RotatorConstants.kV,
      LauncherConstants.RotatorConstants.kA
    );
    angle = 0.0; // Initialize the arm angle to 0 degrees
    
    // Initialize profiled PID controller
    this.pidController = new ProfiledPIDController(
      LauncherConstants.RotatorConstants.kP,
      LauncherConstants.RotatorConstants.kI,
      LauncherConstants.RotatorConstants.kD,
      new TrapezoidProfile.Constraints(
        LauncherConstants.RotatorConstants.maxVelocity,
        LauncherConstants.RotatorConstants.maxAcceleration
      )
    );
    this.motor = new SparkMax(LauncherConstants.RotatorConstants.CAN, SparkLowLevel.MotorType.kBrushless);
    this.motorSim = new SparkMaxSim(motor, DCMotor.getNEO(1));
    this.encoder = motor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    angle = encoder.getPosition() * 360; // Convert encoder position to degrees
    currentVelocity = encoder.getVelocity();
    appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    appliedCurrent = motor.getOutputCurrent();
    
    // Publish telemetry
    if (Constants.verbose) {
      SmartDashboard.putNumber("Launcher/Arm/EncoderPosition", angle);
      SmartDashboard.putNumber("Launcher/Arm/EncoderVelocity", currentVelocity);
      SmartDashboard.putNumber("Launcher/Arm/AppliedVoltage", appliedVoltage);
      SmartDashboard.putNumber("Launcher/Arm/AppliedCurrent", appliedCurrent);
    }

  }

  /**
   * Automatically runs the rotation control to the current setpoint if enabled in SmartDashboard.
   * Stops the motor if system run is disabled.
   */
  public void autoRun() {
    if (systemRun) {
      double targetAngle = setpoint;
      setAngle(targetAngle);
    } else {
      stop();
    }
  }

  /** Returns a command that continually executes autoRun(). */
  public Command autoCommand() {
    return this.run(() -> autoRun());
  }
  
  /**
   * Sets the launcher arm to the target angle, respecting minimum angle bounds.
   * Computes the combined PID and feedforward output.
   * @param targetAngle The desired angle in degrees.
   */
  public void setAngle(double targetAngle) {
    // Ensure the target angle is within the safe range
    targetAngle = Math.max(targetAngle, LauncherConstants.RotatorConstants.minAngle);
    
    // Calculate PID output
    double pid = pidController.calculate(angle, targetAngle);
    SmartDashboard.putNumber("Launcher/Arm/PID/PIDOutput", pid);
    
    // Calculate feedforward using the current encoder velocity and profile setpoint velocity
    double ff = feedforward.calculate(pidController.getSetpoint().position, pidController.getSetpoint().velocity);
    SmartDashboard.putNumber("Launcher/Arm/PID/FeedforwardOutput", ff);
    SmartDashboard.putNumber("Launcher/Arm/PID/TotalOutput", pid + ff);
    
    // Apply voltage to the motor
    motor.setVoltage(pid + ff);
  }

  /** Directly sets the motor voltage. */
  public void setVoltage(double targetVoltage) {
    motor.setVoltage(targetVoltage);
  }

  /** Stops the rotation motor and resets the PID controller based on current angle. */
  public void stop() {
    motor.stopMotor();
    pidController.reset(angle);
  }

  /** Resets the relative encoder position to zero. */
  public void resetEncoder() {
    encoder.setPosition(0);
  }
  
  /** Returns a command that continuously tries to maintain a given angle. */
  public Command runAngle(double Angle) {
    return this.run(() -> setAngle(Angle));
  }

  /** Returns a command that applies a constant voltage. */
  public Command runVoltage(double Voltage) {
    return this.run(() -> setVoltage(Voltage));
  }
  
  /** Returns a one-shot command to reset the encoder position. */
  public Command EncoderResetCommand() {
    return this.runOnce(() -> resetEncoder());
  }

  /** Returns a command that continuously stops the launcher arm. */
  public Command stopCommand() {
    return this.run(this::stop);
  }
}