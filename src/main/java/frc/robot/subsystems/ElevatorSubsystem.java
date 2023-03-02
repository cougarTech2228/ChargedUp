package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.Rev2mDistanceSensor;
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

public class ElevatorSubsystem extends ProfiledPIDSubsystem {

    private ShuffleboardTab m_sbTab;
    private WPI_TalonFX m_elevatorMotor;
    private CT_DigitalInput m_elevatorDownLimit;
    private CT_DigitalInput m_elevatorUpLimit;
    private ElevatorState m_elevatorState = ElevatorState.stopped;
    private double m_feedforwardVal = 0;

    private Rev2mDistanceSensor m_distMxp;
    private double m_elevatorHeight;

    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
    // Steps for tuning the feedforward values Kg and Kv (leave others at 0)
    // 1. Start by setting Kg and Kv to zero.
    // 2. Increase Kg until the arm can hold its position with as little
    // movement as possible. If the arm moves in the opposite direction,
    // decrease until it remains stationary. You will have to zero in on
    // Kg precisely (at least four decimal places).
    // 3. Increase the velocity feedforward gain Kv until the arm tracks
    // the setpoint during smooth, slow motion. If the arm overshoots,
    // reduce the gain. Note that the arm may "lag" the commanded motion,
    // this is normal, and is fine so long as it moves the correct amount
    // in total.

    private static final double kSVolts = 0;
    private static final double kGVolts = 0;
    private static final double kVVolt = 0;
    private static final double kAVolt = 0;

    private static final double kP = 0.35;
    private static final double kI = 0.01;
    private static final double kD = 0.0;
    private static final double kDt = 0.2;

    private static final double kMaxVelocity = 3.0;
    private static final double kMaxAcceleration = 0.75;

    private static final double kMotorVoltageLimit = 12;
    private static final double kPositionErrorTolerance = 0.5; // in cm

    private static final double HEIGHT_MIN = 49.0;

    public static final double HEIGHT_HOME = 50.0;
    public static final double HEIGHT_LOW = 58.0;
    public static final double HEIGHT_PRELOADED_CONE = 65.0;
    public static final double HEIGHT_MIDDLE = 85.0;
    public static final double HEIGHT_HIGH = 87.0;
    public static final double HEIGHT_SHELF = 85.0;

    private static final double HEIGHT_MAX = 88.0;

    private static final double kSafeToLeaveHomeHeight = HEIGHT_HOME + 5.0;

    private static final ProfiledPIDController pidController = new ProfiledPIDController(
            kP, kI, kD,
            new TrapezoidProfile.Constraints(
                    kMaxVelocity,
                    kMaxAcceleration),
            kDt);

    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
            kSVolts, kGVolts,
            kVVolt, kAVolt);

    private enum ElevatorState {
        stopped,
        raising,
        lowering
    };

    public ElevatorSubsystem(DistanceSensorSubsystem distanceSensorSubsystem) {
        super(pidController, 0);

        pidController.setTolerance(kPositionErrorTolerance);

        m_elevatorDownLimit = new CT_DigitalInput(Constants.ELEVATOR_LOWER_LIMIT_DIO);
        m_elevatorUpLimit = new CT_DigitalInput(Constants.ELEVATOR_UPPER_LIMIT_DIO);
        m_elevatorMotor = new WPI_TalonFX(Constants.ELEVATOR_MOTOR_ID);
        m_elevatorMotor.setNeutralMode(NeutralMode.Brake);
        m_elevatorMotor.setInverted(true);

        m_distMxp = distanceSensorSubsystem.getElevatorSensor();

        m_sbTab = Shuffleboard.getTab("Elevator");

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
                return m_elevatorMotor.getMotorOutputVoltage();
            };
        });

        m_sbTab.addDouble("Current Height:", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return m_elevatorHeight;
            };
        });

        m_sbTab.addDouble("FF:", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return m_feedforwardVal;
            };
        });

        m_sbTab.addBoolean("Low Limit", new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return isElevatorLowerLimitReached();
            };
        });
    }

    @Override
    public void periodic() {
        super.periodic();

        if (m_distMxp.isEnabled() && m_distMxp.isRangeValid()) {
            m_elevatorHeight = m_distMxp.getRange(Unit.kMillimeters) / 10.0;

            // Boundary check the distance sensor's range values
            if (m_elevatorHeight > HEIGHT_MAX) {
                System.out.println("Elevator distance sensor exceeded max range limit");
                m_elevatorHeight = HEIGHT_MAX;
            } else if (m_elevatorHeight < HEIGHT_MIN) {
                System.out.println("Elevator distance sensor exceeded min range limit");
                m_elevatorHeight = HEIGHT_MIN;
            }
        }

        if (DriverStation.isDisabled()) {
            pidController.setGoal(getMeasurement());
            disable();
            m_elevatorMotor.set(0);
            return;
        }

        if (isElevatorUpperLimitReached() && (m_elevatorState == ElevatorState.raising)) {
            System.out.println("Elevator Upper Limit Reached");
            disable();
            stopElevator();
        } else if (isElevatorLowerLimitReached() && (m_elevatorState == ElevatorState.lowering)) {
            System.out.println("Elevator Lower Limit Reached");
            disable();
            stopElevator();
        }

        if (pidController.atGoal()) {
            stopElevator();
        }
    }

    public boolean atGoal() {
        return pidController.atGoal();
    }

    public boolean isSafeToExtendArm() {
        return (m_elevatorHeight >= kSafeToLeaveHomeHeight);
    }

    private void stopElevator() {
        if (m_elevatorState != ElevatorState.stopped) {
            System.out.println("stopping elevator");
            m_elevatorState = ElevatorState.stopped;
            m_elevatorMotor.stopMotor();
        }
    }

    public void setElevatorPosition(double height) {
        if (height > m_elevatorHeight) {
            m_elevatorState = ElevatorState.raising;
        } else if (height < m_elevatorHeight) {
            m_elevatorState = ElevatorState.lowering;
        } else {
            m_elevatorState = ElevatorState.stopped;
        }

        System.out.println("setting height: " + height);
        this.m_controller.reset(getMeasurement());
        pidController.setGoal(height);
        enable();
    }

    public boolean isElevatorUpperLimitReached() {
        return !m_elevatorUpLimit.get();
    }

    public boolean isElevatorLowerLimitReached() {
        return !m_elevatorDownLimit.get();
    }

    public boolean isStopped() {
        return (m_elevatorState == ElevatorState.stopped);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        m_feedforwardVal = feedforward;
        double newOutput = output + feedforward;
        // Add the feedforward to the PID output to get the motor output

        // clamp the output to a sane range
        double val;
        if (newOutput < 0) {
            val = Math.max(-kMotorVoltageLimit, newOutput);
        } else {
            val = Math.min(kMotorVoltageLimit, newOutput);
        }

        m_elevatorMotor.setVoltage(val);
    }

    @Override
    public double getMeasurement() {
        return m_elevatorHeight;
    }
}
