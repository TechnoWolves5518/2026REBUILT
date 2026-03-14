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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase {
  private double angle; // Current height of the arm in meters
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
    SmartDashboard.putNumber("Arm/Rotator/EncoderPosition", angle);
    SmartDashboard.putNumber("Arm/Rotator/EncoderVelocity", encoder.getVelocity());
    SmartDashboard.putNumber("Arm/Rotator/AppliedCurrent", motor.getOutputCurrent());
  }

  public void throwArm() {
    motor.setVoltage(5);
    setCoastMode();
  }

  public void liftArm() {
    motor.setVoltage(-5);
    setBrakeMode();
  }

  public void stop() {
    motor.stopMotor();
  }
  
  @SuppressWarnings("removal")
  private void setBrakeMode() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(SparkMaxConfig.IdleMode.kBrake);
    motor.configure(config, SparkMax.ResetMode.kNoResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
  }

  @SuppressWarnings("removal")
  private void setCoastMode() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(SparkMaxConfig.IdleMode.kCoast);
    motor.configure(config, SparkMax.ResetMode.kNoResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
  }
  
  public Command runThrow() {
    return this.runEnd(this::throwArm, this::stop);
  }

  public Command runLift() {
    return this.runEnd(this::liftArm, this::stop);
  }

  public Command stopCommand() {
    return this.run(this::stop);
  }
}
