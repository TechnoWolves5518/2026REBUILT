// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Subsystem for controlling the main Arm rotation.
 * This subsystem uses a SparkMax motor controller and an absolute encoder
 * to monitor the arm's angle and apply simple voltage commands for lifting and throwing.
 */
public class Arm extends SubsystemBase {
  private double angle; // Current height of the arm in meters
  private double velocity;
  private double current;
  private SparkMax motor;
  @SuppressWarnings("unused")
  private SparkMaxSim motorSim;
  private SparkAbsoluteEncoder encoder;

  /** Creates a new Arm. */
  public Arm() {
    angle = 0.0; // Initialize the arm angle to 0 degrees
    this.motor = new SparkMax(Constants.IntakeConstants.ArmConstants.CAN, SparkLowLevel.MotorType.kBrushless);
    this.motorSim = new SparkMaxSim(motor, DCMotor.getNEO(1));
    this.encoder = motor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    angle = encoder.getPosition();
    velocity = encoder.getVelocity();
    current = motor.getOutputCurrent();
    
    // Publish telemetry to SmartDashboard
    if (Constants.verbose) {
      SmartDashboard.putNumber("Arm/Rotator/EncoderPosition", angle);
      SmartDashboard.putNumber("Arm/Rotator/EncoderVelocity", velocity);
      SmartDashboard.putNumber("Arm/Rotator/AppliedCurrent", current);
    }

  }

  /** Applies a positive voltage to throw/lower the arm and sets it to coast mode. */
  public void throwArm() {
    motor.setVoltage(Constants.IntakeConstants.ArmConstants.voltsDown);
    setCoastMode();
  }

  /** Applies a negative voltage to lift the arm and sets it to brake mode. */
  public void liftArm() {
    motor.setVoltage(Constants.IntakeConstants.ArmConstants.voltsUp
    );
    setBrakeMode();
  }

  /** Stops the arm motor. */
  public void stop() {
    motor.stopMotor();
  }
  
  /** Configures the motor's idle mode to Brake. */
  private void setBrakeMode() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(SparkMaxConfig.IdleMode.kBrake);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Configures the motor's idle mode to Coast. */
  private void setCoastMode() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(SparkMaxConfig.IdleMode.kCoast);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  
  /** Returns a command that runs the throw action and stops when interrupted. */
  public Command runThrow() {
    return this.runEnd(this::throwArm, this::stop);
  }

  /** Returns a command that runs the lift action and stops when interrupted. */
  public Command runLift() {
    return this.runEnd(this::liftArm, this::stop);
  }

  /** Returns a command that continuously stops the arm. */
  public Command stopCommand() {
    return this.run(this::stop);
  }
}
