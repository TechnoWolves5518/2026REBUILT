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

import frc.robot.Constants.LauncherConstants.FeederConstants;

public class Feeder extends SubsystemBase {
    private final SparkMax m_motor;
    private final SparkMax m_motor2;
    private final RelativeEncoder m_encoder;
    private final RelativeEncoder m_encoder2;
    private final SimpleMotorFeedforward m_feedforward;
    private final SimpleMotorFeedforward m_feedforward2;
    private final ProfiledPIDController m_pidController;
    private final ProfiledPIDController m_pidController2;


    // We store the target RPM here so we can log it in periodic()
    private double m_targetRPM = 0.0;
    private boolean m_running = false;

    public Feeder() {
        m_motor = new SparkMax(FeederConstants.Upper.kMotorID, MotorType.kBrushless);
        m_motor2 = new SparkMax(FeederConstants.Lower.kMotorID, MotorType.kBrushless);
        
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(FeederConstants.Upper.kCurrentLimit);
        config.idleMode(IdleMode.kCoast);
        m_motor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

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
        SmartDashboard.putNumber("Feeder/Upper/kP", FeederConstants.Upper.kP);
        SmartDashboard.putNumber("Feeder/Upper/kI", FeederConstants.Upper.kI);
        SmartDashboard.putNumber("Feeder/Upper/kD", FeederConstants.Upper.kD);
        SmartDashboard.putNumber("Feeder/Upper/kS", FeederConstants.Upper.kS);
        SmartDashboard.putNumber("Feeder/Upper/kV", FeederConstants.Upper.kV);
        SmartDashboard.putNumber("Feeder/Upper/kA", FeederConstants.Upper.kA);
        SmartDashboard.putNumber("Feeder/Upper/Target", 0);
        SmartDashboard.putNumber("Feeder/Upper/Voltage", 0);
        SmartDashboard.putNumber("Feeder/Lower/kP", FeederConstants.Lower.kP);
        SmartDashboard.putNumber("Feeder/Lower/kI", FeederConstants.Lower.kI);
        SmartDashboard.putNumber("Feeder/Lower/kD", FeederConstants.Lower.kD);
        SmartDashboard.putNumber("Feeder/Lower/kS", FeederConstants.Lower.kS);
        SmartDashboard.putNumber("Feeder/Lower/kV", FeederConstants.Lower.kV);
        SmartDashboard.putNumber("Feeder/Lower/kA", FeederConstants.Lower.kA);
        SmartDashboard.putNumber("Feeder/Lower/Target", 0);
        SmartDashboard.putNumber("Feeder/Lower/Voltage", 0);
    }

    double getVelocityGeared() {
        return m_encoder.getVelocity() * FeederConstants.Upper.kGearRatio;
    }

    double getVelocityGeared2() {
        return m_encoder2.getVelocity() * FeederConstants.Lower.kGearRatio;
    }

    // Runs every 20ms
    @Override
    public void periodic() {
        // 1. LIVE PID TUNING
        // Read values from Dashboard. If they differ from current config, update them.
        double p_upper = SmartDashboard.getNumber("Feeder/Upper/kP", 0);
        double i_upper = SmartDashboard.getNumber("Feeder/Upper/kI", 0);
        double d_upper = SmartDashboard.getNumber("Feeder/Upper/kD", 0);
        double s_upper = SmartDashboard.getNumber("Feeder/Upper/kS", 0);
        double v_upper = SmartDashboard.getNumber("Feeder/Upper/kV", 0);
        double a_upper = SmartDashboard.getNumber("Feeder/Upper/kA", 0);
        double p_lower = SmartDashboard.getNumber("Feeder/Lower/kP", 0);
        double i_lower = SmartDashboard.getNumber("Feeder/Lower/kI", 0);
        double d_lower = SmartDashboard.getNumber("Feeder/Lower/kD", 0);
        double s_lower = SmartDashboard.getNumber("Feeder/Lower/kS", 0);
        double v_lower = SmartDashboard.getNumber("Feeder/Lower/kV", 0);
        double a_lower = SmartDashboard.getNumber("Feeder/Lower/kA", 0);

        // Only update if changed (optimization)
        if (p_upper != m_pidController.getP()) m_pidController.setP(p_upper);
        if (i_upper != m_pidController.getI()) m_pidController.setI(i_upper);
        if (d_upper != m_pidController.getD()) m_pidController.setD(d_upper);
        if (s_upper != m_feedforward.getKs()) m_feedforward.setKs(s_upper);
        if (v_upper != m_feedforward.getKv()) m_feedforward.setKv(v_upper);
        if (a_upper != m_feedforward.getKa()) m_feedforward.setKa(a_upper);
        if (p_lower != m_pidController2.getP()) m_pidController2.setP(p_lower);
        if (i_lower != m_pidController2.getI()) m_pidController2.setI(i_lower);
        if (d_lower != m_pidController2.getD()) m_pidController2.setD(d_lower);
        if (s_lower != m_feedforward2.getKs()) m_feedforward2.setKs(s_lower);
        if (v_lower != m_feedforward2.getKv()) m_feedforward2.setKv(v_lower);
        if (a_lower != m_feedforward2.getKa()) m_feedforward2.setKa(a_lower);


        // 2. TELEMETRY FOR ADVANTAGESCOPE
        // Graph these two lines together to see how well you are tracking
        SmartDashboard.putNumber("Feeder/Main/SetpointRPM", m_targetRPM);
        SmartDashboard.putNumber("Feeder/Upper/ActualRPM", getVelocityGeared());
        SmartDashboard.putNumber("Feeder/Lower/ActualRPM", getVelocityGeared2());
        
        // Useful for seeing if you are maxing out your battery (12V)
        SmartDashboard.putNumber("Feeder/Upper/AppliedVolts", m_motor.getAppliedOutput() * m_motor.getBusVoltage());
        SmartDashboard.putNumber("Feeder/Upper/AppliedCurrent", m_motor.getOutputCurrent());
        SmartDashboard.putBoolean("Feeder/Upper/atSetpoint", atSetpoint());
        SmartDashboard.putNumber("Feeder/Upper/PIDSetpoint", m_pidController.getSetpoint().position);
        SmartDashboard.putNumber("Feeder/Upper/PIDAcceleration", m_pidController.getSetpoint().velocity);
        SmartDashboard.putNumber("Feeder/Lower/AppliedVolts", m_motor2.getAppliedOutput() * m_motor2.getBusVoltage());
        SmartDashboard.putNumber("Feeder/Lower/AppliedCurrent", m_motor2.getOutputCurrent());
        SmartDashboard.putBoolean("Feeder/Lower/atSetpoint", atSetpoint2());
        SmartDashboard.putNumber("Feeder/Lower/PIDSetpoint", m_pidController2.getSetpoint().position);
        SmartDashboard.putNumber("Feeder/Lower/PIDAcceleration", m_pidController2.getSetpoint().velocity);
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
        m_motor.setVoltage(SmartDashboard.getNumber("Flywheel/Voltage", 0));
        m_motor2.setVoltage(SmartDashboard.getNumber("Flywheel/Voltage", 0));
    }


    public void stop() {
        m_running = false;
        m_motor.stopMotor();
        m_motor2.stopMotor();
        m_pidController.reset(0);
        m_pidController2.reset(0);
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

    public Command runFlywheelVoltage() {
        return this.runEnd(this::setTargetVoltage, this::stop);
    }
}