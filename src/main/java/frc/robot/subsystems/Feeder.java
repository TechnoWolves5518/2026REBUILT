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

import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants.FeederConstants;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;

public class Feeder extends SubsystemBase {
    private final SparkMax m_motor;
    private final SparkMax m_motor2;
    @SuppressWarnings("unused")
    private final SparkMaxSim m_motorSim;
    @SuppressWarnings("unused")
    private final SparkMaxSim m_motorSim2;
    private final RelativeEncoder m_encoder;
    private final RelativeEncoder m_encoder2;
    private final SimpleMotorFeedforward m_feedforward;
    private final SimpleMotorFeedforward m_feedforward2;
    private final ProfiledPIDController m_pidController;
    private final ProfiledPIDController m_pidController2;

    


    // We store the target RPM here so we can log it in periodic()
    private double m_targetRPM = 0.0;
    private boolean m_running = false;

    @SuppressWarnings("removal")
    public Feeder() {
        m_motor = new SparkMax(FeederConstants.Upper.kMotorID, MotorType.kBrushless);
        m_motor2 = new SparkMax(FeederConstants.Lower.kMotorID, MotorType.kBrushless);
        m_motorSim = new SparkMaxSim(m_motor, DCMotor.getNEO(1));
        m_motorSim2 = new SparkMaxSim(m_motor2, DCMotor.getNEO(1));

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(FeederConstants.Upper.kCurrentLimit);
        config.idleMode(IdleMode.kCoast);
        config.inverted(true);
        m_motor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.smartCurrentLimit(FeederConstants.Lower.kCurrentLimit);
        config2.idleMode(IdleMode.kCoast);
        m_motor2.configure(config2, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        m_encoder = m_motor.getEncoder();
        m_encoder2 = m_motor2.getEncoder();

        m_feedforward = new SimpleMotorFeedforward(
            FeederConstants.Upper.kS,
            FeederConstants.Upper.kV,
            FeederConstants.Upper.kA
        );
        m_feedforward2 = new SimpleMotorFeedforward(
            FeederConstants.Lower.kS,
            FeederConstants.Lower.kV,
            FeederConstants.Lower.kA
        );


        m_pidController = new ProfiledPIDController(
            FeederConstants.Upper.kP,
            FeederConstants.Upper.kI,
            FeederConstants.Upper.kD,
            new TrapezoidProfile.Constraints(
                FeederConstants.Upper.kMaxVelocityRPM,
                FeederConstants.Upper.kMaxAccelerationRPMps
            )
        );
        m_pidController.setTolerance(FeederConstants.Upper.kTargetToleranceRPM);
        m_pidController2 = new ProfiledPIDController(
            FeederConstants.Lower.kP,
            FeederConstants.Lower.kI,
            FeederConstants.Lower.kD,
            new TrapezoidProfile.Constraints(
                FeederConstants.Lower.kMaxVelocityRPM,
                FeederConstants.Lower.kMaxAccelerationRPMps
            )
        );
        m_pidController2.setTolerance(FeederConstants.Lower.kTargetToleranceRPM);

        // INITIALIZE TUNABLE VALUES
        // We put the starting values onto SmartDashboard so they appear immediately
        SmartDashboard.putNumber("Launcher/Feeder/Upper/kP", FeederConstants.Upper.kP);
        SmartDashboard.putNumber("Launcher/Feeder/Upper/kI", FeederConstants.Upper.kI);
        SmartDashboard.putNumber("Launcher/Feeder/Upper/kD", FeederConstants.Upper.kD);
        SmartDashboard.putNumber("Launcher/Feeder/Upper/kS", FeederConstants.Upper.kS);
        SmartDashboard.putNumber("Launcher/Feeder/Upper/kV", FeederConstants.Upper.kV);
        SmartDashboard.putNumber("Launcher/Feeder/Upper/kA", FeederConstants.Upper.kA);
        SmartDashboard.putNumber("Launcher/Feeder/Upper/Voltage", 0);
        SmartDashboard.putNumber("Launcher/Feeder/Lower/kP", FeederConstants.Lower.kP);
        SmartDashboard.putNumber("Launcher/Feeder/Lower/kI", FeederConstants.Lower.kI);
        SmartDashboard.putNumber("Launcher/Feeder/Lower/kD", FeederConstants.Lower.kD);
        SmartDashboard.putNumber("Launcher/Feeder/Lower/kS", FeederConstants.Lower.kS);
        SmartDashboard.putNumber("Launcher/Feeder/Lower/kV", FeederConstants.Lower.kV);
        SmartDashboard.putNumber("Launcher/Feeder/Lower/kA", FeederConstants.Lower.kA);
        SmartDashboard.putNumber("Launcher/Feeder/Lower/Voltage", 0);
        SmartDashboard.putNumber("Launcher/Feeder/Main/Target", 4000);
    }

    double getVelocityGeared() {
        return m_encoder.getVelocity();
    }

    double getVelocityGeared2() {
        return m_encoder2.getVelocity();
    }

    // Runs every 20ms
    @Override
    public void periodic() {
        if (Constants.TUNING_MODE) {
            // 2. TELEMETRY FOR ADVANTAGESCOPE
            // Graph these two lines together to see how well you are tracking
            SmartDashboard.putNumber("Launcher/Feeder/Main/SetpointRPM", m_targetRPM);
            SmartDashboard.putNumber("Launcher/Feeder/Upper/ActualRPM", getVelocityGeared());
            SmartDashboard.putNumber("Launcher/Feeder/Lower/ActualRPM", getVelocityGeared2());
            
            // Useful for seeing if you are maxing out your battery (12V)
            SmartDashboard.putNumber("Launcher/Feeder/Upper/AppliedVolts", m_motor.getAppliedOutput() * m_motor.getBusVoltage());
            SmartDashboard.putNumber("Launcher/Feeder/Upper/AppliedCurrent", m_motor.getOutputCurrent());
            SmartDashboard.putBoolean("Launcher/Feeder/Upper/atSetpoint", atSetpoint());
            SmartDashboard.putNumber("Launcher/Feeder/Upper/PIDSetpoint", m_pidController.getSetpoint().position);
            SmartDashboard.putNumber("Launcher/Feeder/Upper/PIDAcceleration", m_pidController.getSetpoint().velocity);
            SmartDashboard.putNumber("Launcher/Feeder/Lower/AppliedVolts", m_motor2.getAppliedOutput() * m_motor2.getBusVoltage());
            SmartDashboard.putNumber("Launcher/Feeder/Lower/AppliedCurrent", m_motor2.getOutputCurrent());
            SmartDashboard.putBoolean("Launcher/Feeder/Lower/atSetpoint", atSetpoint2());
            SmartDashboard.putNumber("Launcher/Feeder/Lower/PIDSetpoint", m_pidController2.getSetpoint().position);
            SmartDashboard.putNumber("Launcher/Feeder/Lower/PIDAcceleration", m_pidController2.getSetpoint().velocity);
        }
    }

    public boolean atSetpoint() {
        double currentVelocity = getVelocityGeared();
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
    public boolean atSetpoint2() {
        double currentVelocity = getVelocityGeared2();
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
            m_pidController.reset(getVelocityGeared());
            m_pidController2.reset(getVelocityGeared2());
            m_running = true; // Save for logging
        }

        m_targetRPM = targetRPM; // Save for logging
        
        double pidOutput = m_pidController.calculate(getVelocityGeared(), targetRPM);
        double pidOutput2 = m_pidController2.calculate(getVelocityGeared2(), targetRPM);


        // Use the setpoint from the profile (position is RPM, velocity is RPM/s acceleration)
        TrapezoidProfile.State setpoint = m_pidController.getSetpoint();
        double ffOutput = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        TrapezoidProfile.State setpoint2 = m_pidController2.getSetpoint();
        double ffOutput2 = m_feedforward2.calculate(setpoint2.position, setpoint2.velocity);


        m_motor.setVoltage(ffOutput + pidOutput);
        m_motor2.setVoltage(ffOutput2 + pidOutput2);
    }

    public void setTargetVoltage() {
        m_motor.setVoltage(SmartDashboard.getNumber("Launcher/Feeder/Upper/Voltage", 0));
        m_motor2.setVoltage(SmartDashboard.getNumber("Launcher/Feeder/Lower/Voltage", 0));
    }


    public void stop() {
        m_running = false;
        m_motor.stopMotor();
        m_motor2.stopMotor();
        m_pidController.reset(0);
        m_pidController2.reset(0);
        m_targetRPM = 0.0;
    }

    public Command runFeeder(double targetRPM) {
        return this.runEnd(
            () -> setTargetVelocity(targetRPM),
            this::stop
        );
    }

    public Command runFeederSD() {
            return this.runEnd(
                () -> setTargetVelocity(SmartDashboard.getNumber("Launcher/Feeder/Main/Target", 4000)),
                this::stop
            );
        }

    public Command runFeederVoltage() {
        return this.runEnd(this::setTargetVoltage, this::stop);
    }
}