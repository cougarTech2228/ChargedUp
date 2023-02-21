package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.utils.CT_DigitalInput;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Unit;

public class ExtendoSubsystem extends ProfiledPIDSubsystem {
    
    private WPI_TalonFX m_extendoMotor;
    private ShuffleboardTab m_sbTab;

    private CT_DigitalInput m_extendoHomeLimit;
    private ExtendoState m_extendoState = ExtendoState.stopped;

    private double m_currentArmReachCm;
    private Rev2mDistanceSensor m_distMxp;

    private static final double kSVolts = 0;
    private static final double kGVolts = -0.2;
    private static final double kVVolt = 0.01;
    private static final double kAVolt = 0.1;

    private static final double kP = 0.3;
    private static final double kI = 1.0;
    private static final double kD = 0.0;

    private static final double kMaxVelocityTicksPerSecond = 15;
    private static final double kMaxAccelerationTicksPerSecSquared = 4;
    private static final double kMotorVoltageLimit = 8.0;
    private static final double kPositionErrorTolerance = 0.1;

    public static final double DISTANCE_BOT = 16;
    public static final double DISTANCE_LOW = 32; 
    public static final double DISTANCE_MIDDLE = 40; 
    public static final double DISTANCE_HIGH = 78; 
    public static final double DISTANCE_SHELF = 40;

    private static final ProfiledPIDController pidController = new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                    kMaxVelocityTicksPerSecond,
                    kMaxAccelerationTicksPerSecSquared));

    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
            kSVolts, kGVolts,
            kVVolt, kAVolt);

    private enum ExtendoState {
        stopped,
        extending,
        retracting
    };

    public ExtendoSubsystem(DistanceSensorSubsystem distanceSensorSubsystem) {
        super(pidController, 0);

        pidController.setTolerance(kPositionErrorTolerance);

        m_extendoHomeLimit = new CT_DigitalInput(Constants.EXTENDO_HOME_LIMIT_DIO);
        m_extendoMotor = new WPI_TalonFX(Constants.EXTENDO_MOTOR_ID);
        m_extendoMotor.setNeutralMode(NeutralMode.Brake);
        m_distMxp = distanceSensorSubsystem.getExtendoArmSensor();

        m_sbTab = Shuffleboard.getTab("Extendo");

        m_sbTab.addBoolean("PID Enabled", new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isEnabled();
            };
        });
        m_sbTab.addDouble("PID goal", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return m_controller.getGoal().position;
            };
        });
        m_sbTab.addDouble("PID output", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return m_extendoMotor.getMotorOutputVoltage();
            };
        });

        m_sbTab.addDouble("Current Distance:", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return getMeasurement();
            };
        });
    }

    @Override
    public void periodic() {
        super.periodic();

        if (m_distMxp.isEnabled() && m_distMxp.isRangeValid()) {
            m_currentArmReachCm = m_distMxp.getRange(Unit.kMillimeters) / 10.0;
        }

        if (DriverStation.isDisabled()) {
            pidController.setGoal(getCurrentArmReachCm());
            disable();
            return;
        }

        if (pidController.atGoal()) {
            stopExtending();
        }

        // Need to handle the special case where we may be commanding the
        // arm to retract passed the limit switch.
        if (isExtendoHomeLimitReached() && (m_extendoState == ExtendoState.retracting)) {
            stopExtending();
            System.out.println("Extendo home limit reached");
        }
    }

    public void goToDistanceCM(double distanceCM) {
        if (distanceCM > m_currentArmReachCm) {
            m_extendoState = ExtendoState.extending;
        } else if (distanceCM < m_currentArmReachCm) {
            m_extendoState = ExtendoState.retracting;
        } else {
            m_extendoState = ExtendoState.stopped;
        }

        System.out.println("setting position: " + distanceCM);
        pidController.setGoal(distanceCM);
        enable();
    }

    public double getCurrentArmReachCm() {
        return m_currentArmReachCm;
    }

    public boolean atGoal() {
        return pidController.atGoal();
    }

    private void stopExtending() {
        if (m_extendoState != ExtendoState.stopped) {
            m_extendoMotor.stopMotor();
            m_extendoState = ExtendoState.stopped;
            System.out.println("stopping extendo arm");
            disable();
        }
    }

    public boolean isExtendoHomeLimitReached() {
        return !m_extendoHomeLimit.get();
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {

        if (m_extendoState != ExtendoState.stopped) {
            // Calculate the feedforward from the sepoint
            double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
            double newOutput = output + feedforward;
            // Add the feedforward to the PID output to get the motor output

            // clamp the output to a sane range
            double val;
            if (newOutput < 0) {
                val = Math.max(-kMotorVoltageLimit, newOutput);
            } else {
                val = Math.min(kMotorVoltageLimit, newOutput);
            }

            if (!((m_extendoState == ExtendoState.retracting) && isExtendoHomeLimitReached())) {
                m_extendoMotor.setVoltage(val);
            }
        }
    }

    @Override
    public double getMeasurement() {
        return getCurrentArmReachCm();
    }
}