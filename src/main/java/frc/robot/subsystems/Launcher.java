package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// REV Robotics Imports (2025/2026 API)
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;

    private final SimpleMotorFeedforward m_feedforward;
    private final PIDController m_pidController;

    public Launcher() {
        // 1. Initialize Motor
        m_motor = new SparkMax(LauncherConstants.kMotorID, MotorType.kBrushless);

        // 2. Configure Motor (New REV Config API)
        // In 2026, we prefer using configuration objects over individual setters
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(LauncherConstants.kCurrentLimit);
        config.idleMode(IdleMode.kCoast); // Flywheels should usually Coast, not Brake
        
        // Apply configuration - this is safer than individual calls
        m_motor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        // 3. Get the Built-in Encoder
        // This automatically links to the internal NEO hall sensors
        m_encoder = m_motor.getEncoder();

        // 4. Initialize Controllers
        m_feedforward = new SimpleMotorFeedforward(
            LauncherConstants.kS,
            LauncherConstants.kV,
            LauncherConstants.kA
        );

        m_pidController = new PIDController(
            LauncherConstants.kP,
            LauncherConstants.kI,
            LauncherConstants.kD
        );
        
        m_pidController.setTolerance(LauncherConstants.kTargetToleranceRPM);
    }

    /**
     * Run the flywheel at a specific RPM using Feedforward + PID.
     */
    public void setTargetVelocity(double targetRPM) {
        // A. Feedforward: Calculates base voltage to sustain this RPM
        double ffOutput = m_feedforward.calculate(targetRPM);

        // B. PID: Calculates adjustment voltage based on error
        // m_encoder.getVelocity() returns RPM by default for NEOs
        double pidOutput = m_pidController.calculate(m_encoder.getVelocity(), targetRPM);

        // C. Apply Combined Voltage
        // setVoltage automagically compensates for battery voltage sag
        m_motor.setVoltage(ffOutput + pidOutput);
    }

    public void stop() {
        m_motor.stopMotor();
        m_pidController.reset();
    }

    public boolean isAtSpeed() {
        return m_pidController.atSetpoint();
    }

    public double getVelocityRPM() {
        return m_encoder.getVelocity();
    }

    public Command runFlywheelCommand(double targetRPM) {
        return this.runEnd(
            () -> setTargetVelocity(targetRPM),
            this::stop
        );
    }
}