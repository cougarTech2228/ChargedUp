package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
    WPI_TalonFX m_extendoMotor;

    private ShuffleboardTab m_sbTab;
    
    private CT_DigitalInput m_extendoHomeLimit;
    private ExtendoState extendoState = ExtendoState.stopped;

    private double m_currentArmReachCm;
    private Rev2mDistanceSensor m_distMxp;
    private static final double kSVolts = 0;
    private static final double kGVolts = -.2;
    private static final double kVVolt = 0.01;
    private static final double kAVolt = 0.1;
    private static final double kP = 0.3;
    private static final double kMaxVelocityTicksPerSecond = 15;
    private static final double kMaxAccelerationTicksPerSecSquared = 4;
    public static final double DISTANCE_BOT = 22.5;
    private static final ProfiledPIDController pidController = new ProfiledPIDController(
                kP,
                1,
                0,
                new TrapezoidProfile.Constraints(
                    kMaxVelocityTicksPerSecond,
                    kMaxAccelerationTicksPerSecSquared));
    
    private final ElevatorFeedforward m_feedforward =
        new ElevatorFeedforward(
            kSVolts, kGVolts,
            kVVolt, kAVolt);

    private enum ExtendoState {
        stopped,
        extending,
        retracting
    };

    public ExtendoSubsystem(DistanceSensorSubsystem distanceSensorSubsystem) {
        super(pidController, 0);
        pidController.setTolerance(0.1);
        m_extendoHomeLimit = new CT_DigitalInput(Constants.EXTENDO_HOME_LIMIT_DIO);
        m_extendoMotor = new WPI_TalonFX(Constants.EXTENDO_MOTOR_ID);
        m_extendoMotor.setNeutralMode(NeutralMode.Brake);
        m_distMxp = distanceSensorSubsystem.getExtendoArmSensor();

        m_extendoHomeLimit.setMethodToRun(() -> { // home limit hit
            stopExtending();
        });

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
        if(DriverStation.isDisabled()){
            pidController.setGoal(getCurrentArmReachCm());
            disable();
            return;
        }

        if(isExtendoHomeLimitReached() && extendoState == ExtendoState.retracting){
            stopExtending();
            extendoState = ExtendoState.stopped;
            System.out.println("Extendo home limit reached");
        }
    }

    public void goToDisanceCM(double distanceCM){
        System.out.println("setting position: " + distanceCM);
        pidController.setGoal(distanceCM);
        enable();
    }

    public double getCurrentArmReachCm() {
        return m_currentArmReachCm;
    }

    public boolean atGoal(){
        return pidController.atGoal();
    }

    private void stopExtending(){
        System.out.println("stopping extendo arm");
        m_extendoMotor.stopMotor();
    }

    public void extendArm() {
        if(extendoState != ExtendoState.extending){
            extendoState = ExtendoState.extending;
            m_extendoMotor.set(ControlMode.PercentOutput, Constants.EXTENDO_MOTOR_SPEED);
        } else {
            stopExtending();
        }
    }

    public void retractArm() {
        if(extendoState != ExtendoState.retracting){
            extendoState = ExtendoState.retracting;
            m_extendoMotor.set(ControlMode.PercentOutput, -Constants.EXTENDO_MOTOR_SPEED);
        } else {
            stopExtending();
        }
    }

    public boolean isExtendoHomeLimitReached() {
        return !m_extendoHomeLimit.get();
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        double newOutput = output + feedforward;
        // Add the feedforward to the PID output to get the motor output

        // clamp the output to a sane range
        double val;
        if (newOutput < 0) {
            val = Math.max(-8, newOutput);
        } else {
            val = Math.min(8, newOutput);
        }

        if(!(extendoState == ExtendoState.retracting && isExtendoHomeLimitReached())){
            m_extendoMotor.setVoltage(val);
        }

        if(val > 0){
            extendoState = ExtendoState.extending;
        } else if(val < 0){
            extendoState = ExtendoState.retracting;
        } else{
            extendoState = ExtendoState.stopped;
        }
    }

    @Override
        public double getMeasurement() {
            return getCurrentArmReachCm();
    }
}