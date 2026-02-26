// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;
import swervelib.math.Matter;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final boolean hasFlywheel = false;

  

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class VisionSystem {
      public static final double cameraSTDmultiplier = 4;
  }

  public static class OperatorConstants
  {
    // Xbox Controllers
    public static final int DRIVER_XBOX = 0;
    public static final int SCHMO_XBOX = 1;

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final double FLYWHEEL_RATE = 2000;
  }

  public static final class LauncherConstants {
    public static final class FlywheelConstants {
      public static final int kMotorID = 15; // Set your actual CAN ID
      public static final int kCurrentLimit = 80; // Amps

      // Feedforward Gains (Must be tuned for RPM units)
      // kS: Volts to overcome static friction
      // kV: Volts per RPM (e.g., 12V / 5676 RPM ≈ 0.0021)
      // kA: Volts per (RPM/s) acceleration
      public static final double kS = 0.15; 
      // 1/459
      public static final double kV = 0.0021;
      // 1/500
      public static final double kA = 0.0005;

      // PID Gains
      public static final double kP = 0.006;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static final double kTargetToleranceRPM = 100.0;

      public static final double kMaxVelocityRPM = 5000.0;
      public static final double kMaxAccelerationRPMps = 5000.0;
    }
        
  }
  public static final class IntakeConstants {
    public static final class ArmConstants {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      
      public static final double kS = 0;
      public static final double kG = 0;
      public static final double kV = 0;
      public static final double kA = 0;

      public static final int CAN = 0;

    }
    public static final class FlywheelConstants {
      public static final int kMotorID = 16
      ; // Set your actual CAN ID
      public static final int kCurrentLimit = 80; // Amps

      // Feedforward Gains (Must be tuned for RPM units)
      // kS: Volts to overcome static friction
      // kV: Volts per RPM (e.g., 12V / 5676 RPM ≈ 0.0021)
      // kA: Volts per (RPM/s) acceleration
      public static final double kS = 0.15; 
      // 1/459
      public static final double kV = 0.0021;
      // 1/500
      public static final double kA = 0.0005;

      // PID Gains
      public static final double kP = 0.006;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static final double kTargetToleranceRPM = 100.0;

      public static final double kMaxVelocityRPM = 5000.0;
      public static final double kMaxAccelerationRPMps = 5000.0;
    }
  }
}
