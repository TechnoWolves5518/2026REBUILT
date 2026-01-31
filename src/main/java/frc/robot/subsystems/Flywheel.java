package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Import for Telemetry
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.LauncherConstants;

public class Flywheel extends SubsystemBase {
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SimpleMotorFeedforward m_feedforward;
    private final PIDController m_pidController;

    // We store the target RPM here so we can log it in periodic()
    private double m_targetRPM = 0.0;

    public Flywheel() {
        m_motor = new SparkMax(LauncherConstants.FlywheelConstants.kMotorID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(LauncherConstants.FlywheelConstants.kCurrentLimit);
        config.idleMode(IdleMode.kCoast);
        m_motor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        m_encoder = m_motor.getEncoder();

        m_feedforward = new SimpleMotorFeedforward(
            LauncherConstants.FlywheelConstants.kS,
            LauncherConstants.FlywheelConstants.kV,
            LauncherConstants.FlywheelConstants.kA
        );

        m_pidController = new PIDController(
            LauncherConstants.FlywheelConstants.kP,
            LauncherConstants.FlywheelConstants.kI,
            LauncherConstants.FlywheelConstants.kD
        );
        m_pidController.setTolerance(LauncherConstants.FlywheelConstants.kTargetToleranceRPM);

        // INITIALIZE TUNABLE VALUES
        // We put the starting values onto SmartDashboard so they appear immediately
        SmartDashboard.putNumber("Flywheel/Tuning/kP", LauncherConstants.FlywheelConstants.kP);
        SmartDashboard.putNumber("Flywheel/Tuning/kI", LauncherConstants.FlywheelConstants.kI);
        SmartDashboard.putNumber("Flywheel/Tuning/kD", LauncherConstants.FlywheelConstants.kD);
        SmartDashboard.putNumber("Flywheel/Tuning/kV", LauncherConstants.FlywheelConstants.kV);
        SmartDashboard.putNumber("Flywheel/Target", 0);
    }

    // Runs every 20ms
    @Override
    public void periodic() {
        // 1. LIVE PID TUNING
        // Read values from Dashboard. If they differ from current config, update them.
        double p = SmartDashboard.getNumber("Flywheel/Tuning/kP", 0);
        double i = SmartDashboard.getNumber("Flywheel/Tuning/kI", 0);
        double d = SmartDashboard.getNumber("Flywheel/Tuning/kD", 0);

        // Only update if changed (optimization)
        if (p != m_pidController.getP()) m_pidController.setP(p);
        if (i != m_pidController.getI()) m_pidController.setI(i);
        if (d != m_pidController.getD()) m_pidController.setD(d);

        // 2. TELEMETRY FOR ADVANTAGESCOPE
        // Graph these two lines together to see how well you are tracking
        SmartDashboard.putNumber("Flywheel/SetpointRPM", m_targetRPM);
        SmartDashboard.putNumber("Flywheel/ActualRPM", m_encoder.getVelocity());
        
        // Useful for seeing if you are maxing out your battery (12V)
        SmartDashboard.putNumber("Flywheel/AppliedVolts", m_motor.getAppliedOutput() * m_motor.getBusVoltage());
    }

    public void setTargetVelocity(double targetRPM) {
        m_targetRPM = targetRPM; // Save for logging
        
        double ffOutput = m_feedforward.calculate(targetRPM);
        double pidOutput = m_pidController.calculate(m_encoder.getVelocity(), targetRPM);

        m_motor.setVoltage(ffOutput + pidOutput);
    }

    public void stop() {
        m_motor.stopMotor();
        m_pidController.reset();
        m_targetRPM = 0.0;
    }

    public Command runFlywheelCommand(double targetRPM) {
        return this.runEnd(
            () -> setTargetVelocity(targetRPM),
            this::stop
        );
    }

    public Command runFlywheelCommandSD() {
            return this.runEnd(
                () -> setTargetVelocity(SmartDashboard.getNumber("Flywheel/Target", 0)),
                this::stop
            );
        }
}