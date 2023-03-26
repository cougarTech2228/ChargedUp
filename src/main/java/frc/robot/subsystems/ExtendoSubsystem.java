package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import frc.robot.RobotContainer;
import frc.robot.utils.CT_DigitalInput;

public class ExtendoSubsystem extends ProfiledPIDSubsystem {

    private PneumaticSubsystem m_pneumaticSubsystem;

    private WPI_TalonFX m_extendoMotor;
    private ShuffleboardTab m_sbTab;

    private CT_DigitalInput m_extendoHomeLimit;
    private ExtendoState m_extendoState = ExtendoState.stopped;

    private double m_currentArmReach;

    private static final double kSVolts = 0;
    private static final double kGVolts = 0; //-0.5;
    private static final double kVVolt = 0;
    private static final double kAVolt = 0;

    private static final double kP = 0.1;// 0.5;
    private static final double kI = 0.0;// 0.2;
    private static final double kD = 0.0;
    private static final double kDt = 0.2;

    private static final double kMaxVelocityTicksPerSecond = 200;
    private static final double kMaxAccelerationTicksPerSecSquared = 20;
    private static final double kMotorVoltageLimit = 12.0;

    private static final double kPositionErrorTolerance = 5.0;

    private static final double MIN_DISTANCE = -50.0;

    public static final double DISTANCE_TRANSIT = 12.5;
    public static final double DISTANCE_HOME = -30.0;
    public static final double DISTANCE_LOW = 160.0;
    public static final double DISTANCE_MIDDLE = 230.0;
    public static final double DISTANCE_HIGH = 875.0;
    public static final double DISTANCE_SHELF = 12.5;
    public static final double DISTANCE_CUBE = -30.0;

    private static final double MAX_DISTANCE = 950.0;

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

    public ExtendoSubsystem(PneumaticSubsystem pneumaticSubsystem) {
        super(pidController, 0);

        m_pneumaticSubsystem = pneumaticSubsystem;

        pidController.setTolerance(kPositionErrorTolerance);

        m_extendoHomeLimit = new CT_DigitalInput(Constants.EXTENDO_HOME_LIMIT_DIO);
        m_extendoMotor = new WPI_TalonFX(Constants.EXTENDO_MOTOR_ID);
        m_extendoMotor.setNeutralMode(NeutralMode.Brake);

        resetSensorPosition();

        m_sbTab = Shuffleboard.getTab("Extendo (Debug)");

        m_sbTab.addBoolean("PID Enabled", new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isEnabled();
            };
        });

        m_sbTab.addBoolean("Home", new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isExtendoHomeLimitReached();
            };
        });

        m_sbTab.addString("Extendo State", new Supplier<String>() {
            @Override
            public String get() {
                return m_extendoState.toString();
            };
        });

        m_sbTab.addDouble("PID goal", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return m_controller.getGoal().position;
            };
        });

        m_sbTab.addBoolean("At Goal", new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return atGoal();
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

        m_currentArmReach = m_extendoMotor.getSelectedSensorPosition() / 100.0;
        

        if (m_currentArmReach > MAX_DISTANCE) {
            System.out.println("Extendo distance sensor exceeded max range limit");
            m_currentArmReach = MAX_DISTANCE;
        } else if (m_currentArmReach < MIN_DISTANCE) {
            System.out.println("Extendo distance sensor exceeded min range limit");
            m_currentArmReach = MIN_DISTANCE;
        }

        if (DriverStation.isDisabled()) {
            pidController.setGoal(getCurrentArmReach());
            disable();
            return;
        }

        // if (pidController.atGoal()) {
        //     stopExtending();
        // }

        // Need to handle the special case where we may be commanding the
        // arm to retract passed the limit switch.
        //System.out.println("Extendo State: " + m_extendoState);

        if (isExtendoHomeLimitReached()) {
            resetSensorPosition();
        }

        if (isExtendoHomeLimitReached() && (m_extendoState == ExtendoState.retracting)) {
            stopExtending();
            disable();
            m_pneumaticSubsystem.closeArmBrake();
            m_extendoState = ExtendoState.stopped;
            System.out.println("Extendo home limit reached");
        }

        if (pidController.atGoal()) {
            stopExtending();
            m_pneumaticSubsystem.closeArmBrake();
            disable();
        }
    }

    public void resetSensorPosition() {
        m_extendoMotor.setSelectedSensorPosition(0.0);
    }

    public void retractToHomePosition() {
        // If the arm is safely high enough to reel in to its home position ...
        if (RobotContainer.getElevatorSubsystem().getMeasurement() >= ElevatorSubsystem.HEIGHT_HOME) {
            m_pneumaticSubsystem.closeArmBrake();
            // Rewind the motor so until it hits the proximity sensor
            m_extendoMotor.set(-0.10);
        } else {
            System.out.println("Arm height is not sufficient to home extendo!");
        }
    }

    public void goToDistance(double distance) {
        m_pneumaticSubsystem.openArmBrake();
        // if (distance > m_currentArmReach) {
        //     m_extendoState = ExtendoState.extending;
        // } else if (distance < m_currentArmReach) {
        //     m_extendoState = ExtendoState.retracting;
        // } else {
        //     m_extendoState = ExtendoState.stopped;
        // }

        System.out.println("setting distance: " + distance);
        pidController.setGoal(distance);
        enable();
    }

    public double getCurrentArmReach() {
        return m_currentArmReach;
    }

    public boolean atGoal() {
        return pidController.atGoal();
    }

    public boolean isStopped() {
        return (m_extendoState == ExtendoState.stopped);
    }

    private void stopExtending() {
        m_extendoMotor.stopMotor();
        m_pneumaticSubsystem.closeArmBrake();
        m_extendoMotor.setNeutralMode(NeutralMode.Brake);
        //m_extendoState = ExtendoState.stopped;
        disable();
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
        return getCurrentArmReach();
    }
}