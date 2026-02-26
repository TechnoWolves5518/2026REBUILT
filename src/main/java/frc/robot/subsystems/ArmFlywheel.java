package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Import for Telemetry
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.IntakeConstants;

public class ArmFlywheel extends SubsystemBase {
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SimpleMotorFeedforward m_feedforward;
    private final ProfiledPIDController m_pidController;

    // We store the target RPM here so we can log it in periodic()
    private double m_targetRPM = 0.0;
    private boolean m_running = false;

    public ArmFlywheel() {
        m_motor = new SparkMax(IntakeConstants.FlywheelConstants.kMotorID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(IntakeConstants.FlywheelConstants.kCurrentLimit);
        config.idleMode(IdleMode.kCoast);
        m_motor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        m_encoder = m_motor.getEncoder();

        m_feedforward = new SimpleMotorFeedforward(
            IntakeConstants.FlywheelConstants.kS,
            IntakeConstants.FlywheelConstants.kV,
            IntakeConstants.FlywheelConstants.kA
        );

        m_pidController = new ProfiledPIDController(
            IntakeConstants.FlywheelConstants.kP,
            IntakeConstants.FlywheelConstants.kI,
            IntakeConstants.FlywheelConstants.kD,
            new TrapezoidProfile.Constraints(
                IntakeConstants.FlywheelConstants.kMaxVelocityRPM,
                IntakeConstants.FlywheelConstants.kMaxAccelerationRPMps
            )
        );
        m_pidController.setTolerance(IntakeConstants.FlywheelConstants.kTargetToleranceRPM);

        // INITIALIZE TUNABLE VALUES
        // We put the starting values onto SmartDashboard so they appear immediately
        SmartDashboard.putNumber("ArmFlywheel/kP", IntakeConstants.FlywheelConstants.kP);
        SmartDashboard.putNumber("ArmFlywheel/kI", IntakeConstants.FlywheelConstants.kI);
        SmartDashboard.putNumber("ArmFlywheel/kD", IntakeConstants.FlywheelConstants.kD);
        SmartDashboard.putNumber("ArmFlywheel/kS", IntakeConstants.FlywheelConstants.kS);
        SmartDashboard.putNumber("ArmFlywheel/kV", IntakeConstants.FlywheelConstants.kV);
        SmartDashboard.putNumber("ArmFlywheel/kA", IntakeConstants.FlywheelConstants.kA);
        SmartDashboard.putNumber("ArmFlywheel/Target", 0);
        SmartDashboard.putNumber("ArmFlywheel/Voltage", 0);
    }

    // Runs every 20ms
    @Override
    public void periodic() {
        // 1. LIVE PID TUNING
        // Read values from Dashboard. If they differ from current config, update them.
        double p = SmartDashboard.getNumber("ArmFlywheel/kP", 0);
        double i = SmartDashboard.getNumber("ArmFlywheel/kI", 0);
        double d = SmartDashboard.getNumber("ArmFlywheel/kD", 0);
        double s = SmartDashboard.getNumber("ArmFlywheel/kS", 0);
        double v = SmartDashboard.getNumber("ArmFlywheel/kV", 0);
        double a = SmartDashboard.getNumber("ArmFlywheel/kA", 0);

        // Only update if changed (optimization)
        if (p != m_pidController.getP()) m_pidController.setP(p);
        if (i != m_pidController.getI()) m_pidController.setI(i);
        if (d != m_pidController.getD()) m_pidController.setD(d);
        if (s != m_feedforward.getKs()) m_feedforward.setKs(s);
        if (v != m_feedforward.getKv()) m_feedforward.setKv(v);
        if (a != m_feedforward.getKa()) m_feedforward.setKa(a);


        // 2. TELEMETRY FOR ADVANTAGESCOPE
        // Graph these two lines together to see how well you are tracking
        SmartDashboard.putNumber("ArmFlywheel/SetpointRPM", m_targetRPM);
        SmartDashboard.putNumber("ArmFlywheel/ActualRPM", m_encoder.getVelocity());
        
        // Useful for seeing if you are maxing out your battery (12V)
        SmartDashboard.putNumber("ArmFlywheel/AppliedVolts", m_motor.getAppliedOutput() * m_motor.getBusVoltage());
        SmartDashboard.putNumber("ArmFlywheel/AppliedCurrent", m_motor.getOutputCurrent());
        SmartDashboard.putBoolean("ArmFlywheel/atSetpoint", atSetpoint());
        SmartDashboard.putNumber("ArmFlywheel/PIDSetpoint", m_pidController.getSetpoint().position);
        SmartDashboard.putNumber("ArmFlywheel/PIDAcceleration", m_pidController.getSetpoint().velocity);
    }

    public boolean atSetpoint() {
        double currentVelocity = m_encoder.getVelocity();
        double targetVelocity = m_targetRPM;
        boolean isRunning = m_running;

        if (isRunning) {
            if (Math.abs(currentVelocity - targetVelocity) < 100) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    public void setTargetVelocity(double targetRPM) {
        // If this is the start of the command/ reset the PID controller to start at the current speed.
        if (m_running == false) {
            m_pidController.reset(m_encoder.getVelocity());
            m_running = true; // Save for logging
        }

        m_targetRPM = targetRPM; // Save for logging
        
        double pidOutput = m_pidController.calculate(m_encoder.getVelocity(), targetRPM);

        // Use the setpoint from the profile (position is RPM, velocity is RPM/s acceleration)
        TrapezoidProfile.State setpoint = m_pidController.getSetpoint();
        double ffOutput = m_feedforward.calculate(setpoint.position, setpoint.velocity);

        m_motor.setVoltage(ffOutput + pidOutput);
    }

    public void setTargetVoltage() {
        m_motor.setVoltage(SmartDashboard.getNumber("ArmFlywheel/Voltage", 0));
    }

    public void stop() {
        m_running = false;
        m_motor.stopMotor();
        m_pidController.reset(0);
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
                () -> setTargetVelocity(SmartDashboard.getNumber("ArmFlywheel/Target", 0)),
                this::stop
            );
        }

    public Command runFlywheelVoltage() {
        return this.runEnd(this::setTargetVoltage, this::stop);
    }
}