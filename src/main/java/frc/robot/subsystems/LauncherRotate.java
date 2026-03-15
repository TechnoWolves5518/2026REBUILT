// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;

public class LauncherRotate extends SubsystemBase {
  private double angle; // Current angle of the arm in degrees
  private double setpoint;
  private ProfiledPIDController pidController; // PID controller for arm height control
  private SimpleMotorFeedforward feedforward; // Feedforward for arm control
  private SparkMax motor;
  @SuppressWarnings("unused")
  private SparkMaxSim motorSim;
  private RelativeEncoder encoder;
  /** Creates a new LauncherRotate. */

  public LauncherRotate() {
    SmartDashboard.putNumber("Launcher/Arm/kP", LauncherConstants.RotatorConstants.kP);
    SmartDashboard.putNumber("Launcher/Arm/kI", LauncherConstants.RotatorConstants.kI);
    SmartDashboard.putNumber("Launcher/Arm/kD", LauncherConstants.RotatorConstants.kD);
    SmartDashboard.putNumber("Launcher/Arm/kS", LauncherConstants.RotatorConstants.kS);
    SmartDashboard.putNumber("Launcher/Arm/kV", LauncherConstants.RotatorConstants.kV);
    SmartDashboard.putNumber("Launcher/Arm/kA", LauncherConstants.RotatorConstants.kA);
    SmartDashboard.putBoolean("Launcher/Arm/systemRun", true);
    SmartDashboard.putNumber("Launcher/Arm/Voltage", 0);
    SmartDashboard.putNumber("Launcher/Arm/Current", 0);

    this.setpoint = LauncherConstants.RotatorConstants.defaultSetpoint;
    this.feedforward = new SimpleMotorFeedforward(
      LauncherConstants.RotatorConstants.kS,
      LauncherConstants.RotatorConstants.kV,
      LauncherConstants.RotatorConstants.kA
    );
    angle = 0.0; // Initialize the arm angle to 0 degrees
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
    angle = encoder.getPosition() * 360;
    if (Constants.TUNING_MODE) {
      SmartDashboard.putNumber("Launcher/Arm/EncoderPosition", angle);
      SmartDashboard.putNumber("Launcher/Arm/EncoderVelocity", encoder.getVelocity());
      SmartDashboard.putNumber("Launcher/Arm/AppliedVoltage", motor.getAppliedOutput() * motor.getBusVoltage());
      SmartDashboard.putNumber("Launcher/Arm/AppliedCurrent", motor.getOutputCurrent());
    }
  }

  public void autoRun() {
    if (SmartDashboard.getBoolean("Launcher/Arm/systemRun", true)) {
      double targetAngle = setpoint;
      setAngle(targetAngle);
    } else {
      stop();
    }
  }

  public Command autoCommand() {
    return this.run(() -> autoRun());
  }
  
  public void setAngle(double targetAngle) {
    // Ensure the target angle is within the safe range
    targetAngle = Math.max(targetAngle, LauncherConstants.RotatorConstants.minAngle);
    double pid = pidController.calculate(angle, targetAngle);
    SmartDashboard.putNumber("Launcher/Arm/PID/PIDOutput", pid);
    double ff = feedforward.calculate(pidController.getSetpoint().position, pidController.getSetpoint().velocity);
    SmartDashboard.putNumber("Launcher/Arm/PID/FeedforwardOutput", ff);
    SmartDashboard.putNumber("Launcher/Arm/PID/TotalOutput", pid + ff);
    motor.setVoltage(pid + ff);
  }

  public void setVoltage(double targetVoltage) {
    motor.setVoltage(targetVoltage);
  }

  public void stop() {
    motor.stopMotor();
    pidController.reset(angle);
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }
  
  public Command runAngle(double Angle) {
    return this.run(() -> setAngle(Angle));
  }

  public Command runVoltage(double Voltage) {
    return this.run(() -> setVoltage(Voltage));
  }
  
  public Command EncoderResetCommand() {
    return this.runOnce(() -> resetEncoder());
  }

  public Command stopCommand() {
    return this.run(this::stop);
  }
}
