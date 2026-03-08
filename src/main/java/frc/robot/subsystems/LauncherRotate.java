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
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.Units;

public class LauncherRotate extends SubsystemBase {
  private double angle; // Current height of the arm in meters
  private PIDController pidController; // PID controller for arm height control
  private ArmFeedforward feedforward; // Feedforward for arm control
  private SparkMax motor;
  private RelativeEncoder encoder;
  /** Creates a new LauncherRotate. */

  public LauncherRotate() {
    SmartDashboard.putNumber("Launcher/Arm/kP", LauncherConstants.RotatorConstants.kP);
    SmartDashboard.putNumber("Launcher/Arm/kI", LauncherConstants.RotatorConstants.kI);
    SmartDashboard.putNumber("Launcher/Arm/kD", LauncherConstants.RotatorConstants.kD);
    SmartDashboard.putNumber("Launcher/Arm/kS", LauncherConstants.RotatorConstants.kS);
    SmartDashboard.putNumber("Launcher/Arm/kG", LauncherConstants.RotatorConstants.kG);
    SmartDashboard.putNumber("Launcher/Arm/kV", LauncherConstants.RotatorConstants.kV);
    SmartDashboard.putNumber("Launcher/Arm/kA", LauncherConstants.RotatorConstants.kA);
    SmartDashboard.putNumber("Launcher/Arm/Voltage", 0);
    SmartDashboard.putNumber("Launcher/Arm/Current", 0);
    SmartDashboard.putNumber("Launcher/Arm/Angle", 0);

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
    this.encoder = motor.getEncoder();
    encoder.setPosition(Units.Radians.convertFrom(-20, Units.Degrees));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    angle = Units.Degrees.convertFrom(encoder.getPosition(), Units.Radians);
    SmartDashboard.putNumber("Launcher/Arm/EncoderPosition", angle);
    SmartDashboard.putNumber("Launcher/Arm/EncoderVelocity", encoder.getVelocity());
    SmartDashboard.putNumber("Launcher/Arm/AppliedVoltage", motor.getAppliedOutput() * motor.getBusVoltage());
    SmartDashboard.putNumber("Launcher/Arm/AppliedCurrent", motor.getOutputCurrent());
    
    double kP = SmartDashboard.getNumber("Launcher/Arm/kP", LauncherConstants.RotatorConstants.kP);
    double kI = SmartDashboard.getNumber("Launcher/Arm/kI", LauncherConstants.RotatorConstants.kI);
    double kD = SmartDashboard.getNumber("Launcher/Arm/kD", LauncherConstants.RotatorConstants.kD);
    double kS = SmartDashboard.getNumber("Launcher/Arm/kS", LauncherConstants.RotatorConstants.kS);
    double kG = SmartDashboard.getNumber("Launcher/Arm/kG", LauncherConstants.RotatorConstants.kG);
    double kV = SmartDashboard.getNumber("Launcher/Arm/kV", LauncherConstants.RotatorConstants.kV);
    double kA = SmartDashboard.getNumber("Launcher/Arm/kA", LauncherConstants.RotatorConstants.kA);

    if (kP != pidController.getP()) pidController.setP(kP);
    if (kI != pidController.getI()) pidController.setI(kI);
    if (kD != pidController.getD()) pidController.setD(kD);

    if (kS != feedforward.getKs()) feedforward.setKs(kS);
    if (kG != feedforward.getKg()) feedforward.setKg(kG);
    if (kV != feedforward.getKv()) feedforward.setKv(kV);
    if (kA != feedforward.getKa()) feedforward.setKa(kA);
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
