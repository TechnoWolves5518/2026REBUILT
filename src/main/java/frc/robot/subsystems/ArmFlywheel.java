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

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;

public class ArmFlywheel extends SubsystemBase {
    private final SparkMax m_motor;
    @SuppressWarnings("unused")
    private final SparkMaxSim m_motorSim;
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
        m_motorSim = new SparkMaxSim(m_motor, DCMotor.getNEO(1));

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
        SmartDashboard.putNumber("Arm/Flywheel/kP", IntakeConstants.FlywheelConstants.kP);
        SmartDashboard.putNumber("Arm/Flywheel/kI", IntakeConstants.FlywheelConstants.kI);
        SmartDashboard.putNumber("Arm/Flywheel/kD", IntakeConstants.FlywheelConstants.kD);
        SmartDashboard.putNumber("Arm/Flywheel/kS", IntakeConstants.FlywheelConstants.kS);
        SmartDashboard.putNumber("Arm/Flywheel/kV", IntakeConstants.FlywheelConstants.kV);
        SmartDashboard.putNumber("Arm/Flywheel/kA", IntakeConstants.FlywheelConstants.kA);
        SmartDashboard.putNumber("Arm/Flywheel/Target", IntakeConstants.FlywheelConstants.kTargetRPM);
        SmartDashboard.putNumber("Arm/Flywheel/Voltage", 0);
    }

    // Runs every 20ms
    @Override
    public void periodic() {


        // 2. TELEMETRY FOR ADVANTAGESCOPE
        // Graph these two lines together to see how well you are tracking
        SmartDashboard.putNumber("Arm/Flywheel/SetpointRPM", m_targetRPM);
        SmartDashboard.putNumber("Arm/Flywheel/ActualRPM", m_encoder.getVelocity());
        
        // Useful for seeing if you are maxing out your battery (12V)
        SmartDashboard.putNumber("Arm/Flywheel/AppliedVolts", m_motor.getAppliedOutput() * m_motor.getBusVoltage());
        SmartDashboard.putNumber("Arm/Flywheel/AppliedCurrent", m_motor.getOutputCurrent());
        SmartDashboard.putBoolean("Arm/Flywheel/atSetpoint", atSetpoint());
        SmartDashboard.putNumber("Arm/Flywheel/PIDSetpoint", m_pidController.getSetpoint().position);
        SmartDashboard.putNumber("Arm/Flywheel/PIDAcceleration", m_pidController.getSetpoint().velocity);
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
        
        double pidOutput = m_pidController.calculate((m_encoder.getVelocity() * IntakeConstants.FlywheelConstants.kGearRatio), targetRPM);

        // Use the setpoint from the profile (position is RPM, velocity is RPM/s acceleration)
        TrapezoidProfile.State setpoint = m_pidController.getSetpoint();
        double ffOutput = m_feedforward.calculateWithVelocities(m_encoder.getVelocity(), setpoint.position);

        m_motor.setVoltage(ffOutput + pidOutput);
    }

    public void setTargetVoltage() {
        m_motor.setVoltage(SmartDashboard.getNumber("Arm/Flywheel/Voltage", 0));
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
                () -> setTargetVelocity(-SmartDashboard.getNumber("Arm/Flywheel/Target", 0)),
                this::stop
            );
        }

    public Command runFlywheelVoltage() {
        return this.runEnd(this::setTargetVoltage, this::stop);
    }
}