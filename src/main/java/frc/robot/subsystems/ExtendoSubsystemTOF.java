package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.utils.CT_DigitalInput;

public class ExtendoSubsystemTOF extends ProfiledPIDSubsystem {

    private WPI_TalonFX m_extendoMotor;
    private ShuffleboardTab m_sbTab;

    private CT_DigitalInput m_extendoHomeLimit;
    private ExtendoState m_extendoState = ExtendoState.stopped;

    private double m_currentArmReachCm;
    private Rev2mDistanceSensor m_distMxp;

    private static final double kSVolts = 0;
    private static final double kGVolts = -0.9;// -0.2;
    private static final double kVVolt = 0;// 0.01;
    private static final double kAVolt = 0;// 0.1;

    private static final double kP = 0.5;
    private static final double kI = 0.2;
    private static final double kD = 0.0;
    private static final double kDt = 0.2;

    private static final double kMaxVelocityTicksPerSecond = 8;
    private static final double kMaxAccelerationTicksPerSecSquared = 1.5;
    private static final double kMotorVoltageLimit = 12.0;
    private static final double kPositionErrorTolerance = 2.0; // in cm

    private static final double MIN_DISTANCE = 9;

    public static final double DISTANCE_TRANSIT = 12.5;
    public static final double DISTANCE_HOME = 12.5;
    public static final double DISTANCE_LOW = 35;
    public static final double DISTANCE_MIDDLE = 40;
    public static final double DISTANCE_HIGH = 81;
    public static final double DISTANCE_SHELF = 30;

    private static final double MAX_DISTANCE = 98;
    
    private double m_feedforwardVal = 0;

    private static final ProfiledPIDController pidController = new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                    kMaxVelocityTicksPerSecond,
                    kMaxAccelerationTicksPerSecSquared),
            kDt);

    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
            kSVolts, kGVolts,
            kVVolt, kAVolt);

    private enum ExtendoState {
        stopped,
        extending,
        retracting
    };

    public ExtendoSubsystemTOF() {
        super(pidController, 0);

        pidController.setTolerance(kPositionErrorTolerance);

        m_extendoHomeLimit = new CT_DigitalInput(Constants.EXTENDO_HOME_LIMIT_DIO);
        m_extendoMotor = new WPI_TalonFX(Constants.EXTENDO_MOTOR_ID);
        m_extendoMotor.setNeutralMode(NeutralMode.Brake);

        m_distMxp = new Rev2mDistanceSensor(Port.kMXP);
        m_distMxp.setAutomaticMode(true);

        m_sbTab = Shuffleboard.getTab("Extendo (Debug)");

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

        m_sbTab.addDouble("FF:", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return m_feedforwardVal;
            };
        });

        m_sbTab.addDouble("Motor Voltage:", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return m_extendoMotor.getMotorOutputVoltage();
            };
        });
    }

    @Override
    public void periodic() {
        super.periodic();

        if (m_distMxp.isRangeValid()) {
            m_currentArmReachCm = m_distMxp.getRange(Unit.kMillimeters) / 10.0;

            // Boundary check the distance sensor's range values
            if (m_currentArmReachCm > MAX_DISTANCE) {
                System.out.println("Extendo distance sensor exceeded max range limit");
                m_currentArmReachCm = MAX_DISTANCE;
            } else if (m_currentArmReachCm < MIN_DISTANCE) {
                System.out.println("Extendo distance sensor exceeded min range limit");
                m_currentArmReachCm = MIN_DISTANCE;
            }
        }

        if (DriverStation.isDisabled()) {
            pidController.setGoal(getCurrentArmReachCm());
            disable();
            return;
        }

        // if (pidController.atGoal()) {
        // stopExtending();
        // }

        // Need to handle the special case where we may be commanding the
        // arm to retract passed the limit switch.
        if (isExtendoHomeLimitReached() && (m_extendoState == ExtendoState.retracting)) {
            stopExtending();
            // disable();
            System.out.println("Extendo home limit reached");
        }
    }

    public void goToDistance(double distanceCM) {
        System.out.println("setting distance: " + distanceCM);
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
        m_extendoMotor.stopMotor();
        m_extendoMotor.setNeutralMode(NeutralMode.Brake);
        m_extendoState = ExtendoState.stopped;
        System.out.println("stopping extendo arm");
    }

    public boolean isExtendoHomeLimitReached() {
        return !m_extendoHomeLimit.get();
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {

        // Calculate the feedforward from the setpoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        m_feedforwardVal = feedforward;
        double newOutput = output + feedforward;
        // Add the feedforward to the PID output to get the motor output

        if (output < 0) {
            m_extendoState = ExtendoState.retracting;
        } else if (output > 0) {
            m_extendoState = ExtendoState.extending;
        } else {
            m_extendoState = ExtendoState.stopped;
        }
        
        // clamp the output to a sane range
        double val;
        if (newOutput < 0) {
            val = Math.max(-kMotorVoltageLimit, newOutput);
        } else {
            val = Math.min(kMotorVoltageLimit, newOutput);
        }

        m_extendoMotor.setVoltage(val);
    }

    @Override
    public double getMeasurement() {
        return getCurrentArmReachCm();
    }
}