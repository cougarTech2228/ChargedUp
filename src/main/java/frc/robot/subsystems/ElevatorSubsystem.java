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
    WPI_TalonFX m_elevatorMotor;
    private CT_DigitalInput m_elevatorDownLimit;
    private CT_DigitalInput m_elevatorUpLimit;
    private ElevatorState elevatorState = ElevatorState.stopped;
    private double m_feedforwardVal = 0;

    private Rev2mDistanceSensor m_distMxp;
    private double elevatorHight;

    private static final double kSVolts = 0;
    private static final double kGVolts = 0;
    private static final double kVVolt = 0;
    private static final double kAVolt = 0;
    private static final double kP = 0.5;
    private static final double kI = 1.5;
    private static final double kD = 0;
    private static final double kMaxVelocity = 10;
    private static final double kMaxAcceleration = 2;
    private static final double kMotorVoltageLimit = 8.0;
    private static final double kPositionErrorTolerance = 0.1;

    public static final double DISTANCE_BOT = 0.0;
  
    private static final ProfiledPIDController pidController = new ProfiledPIDController(
            kP, kI, kD,
            new TrapezoidProfile.Constraints(
                    kMaxVelocity,
                    kMaxAcceleration));

    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
            kSVolts, kGVolts,
            kVVolt, kAVolt);

    private enum ElevatorState {
        stopped,
        raising,
        lowering
    };

    public enum ElevatorPosition {
        floor,
        middle,
        top,
        unknown
    };

    public ElevatorSubsystem(DistanceSensorSubsystem distanceSensorSubsystem) {
        super(pidController, 0);

        pidController.setTolerance(kPositionErrorTolerance);

        m_elevatorDownLimit = new CT_DigitalInput(Constants.ELEVATOR_LOWER_LIMIT_DIO);
        m_elevatorUpLimit = new CT_DigitalInput(Constants.ELEVATOR_UPPER_LIMIT_DIO);
        m_elevatorMotor = new WPI_TalonFX(Constants.ELEVATOR_MOTOR_ID);
        m_elevatorMotor.setNeutralMode(NeutralMode.Brake);

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
                return elevatorHight;
            };
        });

        m_sbTab.addDouble("FF:", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return m_feedforwardVal;
            };
        });
    }

    @Override
    public void periodic() {
        super.periodic();

        if (m_distMxp.isEnabled() && m_distMxp.isRangeValid()) {
            elevatorHight = m_distMxp.getRange(Unit.kMillimeters) / 10.0;
        }

        if (DriverStation.isDisabled()) {
            pidController.setGoal(getMeasurement());
            disable();
            m_elevatorMotor.set(0);
            return;
        }

        if (isElevatorUpperLimitReached() && (elevatorState == ElevatorState.raising)) {
            stopElevator();
        } else if (isElevatorLowerLimitReached() && (elevatorState == ElevatorState.lowering)) {
            stopElevator();
        }

        if (isElevatorLowerLimitReached()) {
            m_elevatorMotor.setSelectedSensorPosition(0);
        }

        if (isElevatorUpperLimitReached()) {
            System.out.println(m_elevatorMotor.getSelectedSensorPosition());
            System.out.println("Elevator Upper Limit Reached"); // KAS DEBUG
        }
    }

    public boolean atGoal() {
        return pidController.atGoal();
    }

    private void stopElevator() {
        System.out.println("stopping elevator");
        elevatorState = ElevatorState.stopped;
        m_elevatorMotor.stopMotor();
    }

    public void setElevatorPosition(double height) {
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

        if (val > 0) {
            elevatorState = ElevatorState.raising;
        } else if (val < 0) {
            elevatorState = ElevatorState.lowering;
        } else {
            elevatorState = ElevatorState.stopped;
        }
    }

    @Override
    public double getMeasurement() {
        return elevatorHight;
    }
}
