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
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;

import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Subsystem for controlling the main launcher flywheel.
 * Utilizes a SparkMax motor, relative encoder, ProfiledPIDController,
 * and SimpleMotorFeedforward to accurately achieve and maintain target RPMs.
 */
public class Flywheel extends SubsystemBase {
    private final SparkMax m_motor;
    @SuppressWarnings("unused")
    private final SparkMaxSim m_motorSim;
    private final RelativeEncoder m_encoder;
    private final SimpleMotorFeedforward m_feedforward;
    private final ProfiledPIDController m_pidController;

    @AutoLogOutput(key="Launcher/Flywheel/ActualRPM")
    private double flywheelVelocity;
    @AutoLogOutput(key="Launcher/Flywheel/AppliedVolts")
    private double appliedVolts;
    @AutoLogOutput(key="Launcher/Flywheel/AppliedCurrent")
    private double appliedCurrent;
    @AutoLogOutput(key="Launcher/Flywheel/atSetpoint")
    private boolean atSetpoint;
    @AutoLogOutput(key="Launcher/Flywheel/PIDSetpoint")
    private double pidSetpoint;
    @AutoLogOutput(key="Launcher/Flywheel/PIDAcceleration")
    private double pidAcceleration;

    // We store the target RPM here so we can log it in periodic()
    
    @AutoLogOutput(key="Launcher/Flywheel/SetpointRPM")
    private double m_targetRPM = 0.0;
    private boolean m_running = false;

    /** Creates a new Flywheel subsystem. */
    public Flywheel() {
        m_motor = new SparkMax(LauncherConstants.FlywheelConstants.kMotorID, MotorType.kBrushless);
        m_motorSim = new SparkMaxSim(m_motor, DCMotor.getNEO(1));
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(LauncherConstants.FlywheelConstants.kCurrentLimit);
        config.idleMode(IdleMode.kCoast);
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_motor.getEncoder();

        // Initialize feedforward for predictive voltage calculation
        m_feedforward = new SimpleMotorFeedforward(
            LauncherConstants.FlywheelConstants.kS,
            LauncherConstants.FlywheelConstants.kV,
            LauncherConstants.FlywheelConstants.kA
        );

        // Initialize profiled PID controller for smooth acceleration/deceleration
        m_pidController = new ProfiledPIDController(
            LauncherConstants.FlywheelConstants.kP,
            LauncherConstants.FlywheelConstants.kI,
            LauncherConstants.FlywheelConstants.kD,
            new TrapezoidProfile.Constraints(
                LauncherConstants.FlywheelConstants.kMaxVelocityRPM,
                LauncherConstants.FlywheelConstants.kMaxAccelerationRPMps
            )
        );
        m_pidController.setTolerance(LauncherConstants.FlywheelConstants.kTargetToleranceRPM);

        // INITIALIZE TUNABLE VALUES
        // We put the starting values onto SmartDashboard so they appear immediately
        SmartDashboard.putNumber("Launcher/Flywheel/Voltage", 0);
    }

    // Runs every 20ms
    @Override
    public void periodic() {
        // 2. TELEMETRY FOR ADVANTAGESCOPE
        // Graph these two lines together to see how well you are tracking
        if (Constants.verbose) {
            SmartDashboard.putNumber("Launcher/Flywheel/SetpointRPM", m_targetRPM);
            SmartDashboard.putNumber("Launcher/Flywheel/ActualRPM", m_encoder.getVelocity());
        }
        flywheelVelocity = m_encoder.getVelocity();

        // Useful for seeing if you are maxing out your battery (12V)
        if (Constants.verbose) {
            SmartDashboard.putNumber("Launcher/Flywheel/AppliedVolts", m_motor.getAppliedOutput() * m_motor.getBusVoltage());
            SmartDashboard.putNumber("Launcher/Flywheel/AppliedCurrent", m_motor.getOutputCurrent());
            SmartDashboard.putBoolean("Launcher/Flywheel/atSetpoint", atSetpoint());
            SmartDashboard.putNumber("Launcher/Flywheel/PIDSetpoint", m_pidController.getSetpoint().position);
            SmartDashboard.putNumber("Launcher/Flywheel/PIDAcceleration", m_pidController.getSetpoint().velocity);
        }

        appliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        appliedCurrent = m_motor.getOutputCurrent();
        atSetpoint = atSetpoint();
        pidSetpoint = m_pidController.getSetpoint().position;
        pidAcceleration = m_pidController.getSetpoint().velocity;
    }

    /** 
     * Checks if the flywheel is currently running and at the target RPM within a 100 RPM tolerance.
     * @return true if at setpoint, false otherwise.
     */
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

    /**
     * Sets the target velocity for the flywheel in RPM.
     * Combines PID calculation with feedforward based on the trapezoidal profile setpoint.
     * @param targetRPM The desired flywheel speed in RPM.
     */
    public void setTargetVelocity(double targetRPM) {
        // If this is the start of the command/ reset the PID controller to start at the current speed.
        if (m_running == false) {
            m_pidController.reset(m_encoder.getVelocity());
            m_running = true; // Save for logging
        }

        m_targetRPM = targetRPM; // Save for logging
        
        // Calculate PID output
        double pidOutput = m_pidController.calculate(m_encoder.getVelocity(), targetRPM);

        // Use the setpoint from the profile (position is RPM, velocity is RPM/s acceleration) to calculate feedforward
        TrapezoidProfile.State setpoint = m_pidController.getSetpoint();
        double ffOutput = m_feedforward.calculate(setpoint.position, setpoint.velocity);

        m_motor.setVoltage(ffOutput + pidOutput);
    }

    /** Sets the motor to a direct voltage read from SmartDashboard for testing. */
    public void setTargetVoltage() {
        m_motor.setVoltage(SmartDashboard.getNumber("Launcher/Flywheel/Voltage", 0));
    }

    /** Stops the flywheel and resets state. */
    public void stop() {
        m_running = false;
        m_motor.stopMotor();
        m_pidController.reset(0);
        m_targetRPM = 0.0;
    }

    /**
     * Returns a command that runs the flywheel to a specified RPM and stops it when ended.
     * @param targetRPM The desired target RPM.
     * @return Command for running the flywheel.
     */
    public Command runFlywheelCommand(double targetRPM) {
        return this.runEnd(
            () -> setTargetVelocity(targetRPM),
            this::stop
        );
    }

    /** Returns a command that runs the flywheel using the pre-configured target RPM from constants. */
    public Command runFlywheelCommandSD() {
            return this.runEnd(
                () -> setTargetVelocity(LauncherConstants.FlywheelConstants.kTargetRPM),
                this::stop
            );
        }

    /** Returns a command that runs the flywheel using direct voltage control from SmartDashboard. */
    public Command runFlywheelVoltage() {
        return this.runEnd(this::setTargetVoltage, this::stop);
    }

    public Command stopCommand() {
        return this.runOnce(this::stop);
    }
}
