package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.print.attribute.standard.Destination;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.CT_DigitalInput;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

public class ExtendoSubsystem extends ProfiledPIDSubsystem {
    WPI_TalonFX m_extendoMotor;

    private ShuffleboardTab m_sbTab;
    
    private CT_DigitalInput m_extendoHomeLimit;
    private ExtendoState extendoState = ExtendoState.stopped;

    private double m_currentArmReachCm;
    private Rev2mDistanceSensor m_distMxp;
    public static final double kSVolts = 0;
    public static final double kGVolts = 0;
    public static final double kVVolt = 0.01;
    public static final double kAVolt = 0.1;
    public static final double kP = 0.3;
    public static final double kMaxVelocityTicksPerSecond = 5;
    public static final double kMaxAccelerationTicksPerSecSquared = 4;
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

    public ExtendoSubsystem() {
        super(pidController, 0);
        pidController.setTolerance(0.1);
        m_extendoHomeLimit = new CT_DigitalInput(Constants.EXTENDO_HOME_LIMIT_DIO);
        m_extendoMotor = new WPI_TalonFX(Constants.EXTENDO_MOTOR_ID);
        m_extendoMotor.setNeutralMode(NeutralMode.Brake);
        m_distMxp = new Rev2mDistanceSensor(Port.kMXP);

        m_extendoHomeLimit.setMethodToRun(() -> { // home limit hit
            stopExtending();
        });

        enableDistanceSensor(true);

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
        // We can use timestamp updates to see if the measurement's value is 'stale'
        if (m_distMxp.isEnabled() && m_distMxp.isRangeValid()) {
            // System.out.println(m_distMxp.getTimestamp() + ": Range: " +
            // m_distMxp.getRange());
            m_currentArmReachCm = m_distMxp.getRange(Unit.kMillimeters) / 10.0;
            // System.out.println(getCurrentArmReachCm());
        }

        if (pidController.atGoal()) {
            //System.out.println("At Goal!!!!");
            //disable();
            // m_elevatorMotor.setNeutralMode(NeutralMode.Brake);
            // m_elevatorMotor.set(0);
            // m_elevatorMotor.
        }

        if(isExtendoHomeLimitReached() && extendoState == ExtendoState.retracting){
            stopExtending();
            disable();
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
        // TODO - This is needed from the ButtonBoard to make incremental
        // reach changes with the Joystick
        return m_currentArmReachCm;
    }

    @Override
    public void enable() {
        // TODO Auto-generated method stub
        m_distMxp.setAutomaticMode(true);
        super.enable();
    }

    public void enableDistanceSensor(boolean isEnabled) {
        m_distMxp.setAutomaticMode(isEnabled);
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

        // double scaled = (val / 20) ; // -1V to +1v
        // System.out.println("clamped output: " + val + ", scaled: " + scaled);

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
            double distance = getCurrentArmReachCm();
            return distance;
    }
}