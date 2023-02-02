package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.print.attribute.standard.Destination;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.CT_DigitalInput;

public class ElevatorSubsystem extends ProfiledPIDSubsystem {

    private ShuffleboardTab m_sbTab;
    //TalonFX m_winchMotor = new TalonFX(Constants.ARM_WINCH_MOTOR_ID); //(((******UNSURE IF NEEDED******)))
    WPI_TalonFX m_elevatorMotor;
    private CT_DigitalInput m_elevatorDownLimit;
    private CT_DigitalInput m_elevatorUpLimit;
    private boolean isUpperLimitReached;
    private ElevatorState elevatorState = ElevatorState.stopped;
    // private ElevatorPosition elevatorPosition = ElevatorPosition.unknown;
    // private ElevatorPosition elevatorDestination = ElevatorPosition.unknown;
    // private ElevatorPosition currentElevatorPosition = ElevatorPosition.unknown;

    private static final double kSVolts = 0.1;
    private static final double kGVolts = 0.7;
    private static final double kVVolt = 0.1;
    private static final double kAVolt = 0.1;
    private static final double kP = 0.5;
    private static final double kI = 1;
    private static final double kD = 0;
    private static final double kMaxVelocity = 3;
    private static final double kMaxAcceleration = 2;
    private static final ProfiledPIDController pidController = new ProfiledPIDController(
                kP, kI, kD,
                new TrapezoidProfile.Constraints(
                    kMaxVelocity,
                    kMaxAcceleration));
    
    private final ElevatorFeedforward m_feedforward =
        new ElevatorFeedforward(
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

    public ElevatorSubsystem() {
        super(pidController, 0);
        pidController.setTolerance(0.1);
        //pidController.enableContinuousInput(0, 115000);

        m_elevatorDownLimit = new CT_DigitalInput(Constants.ELEVATOR_LOWER_LIMIT_DIO);
        m_elevatorUpLimit = new CT_DigitalInput(Constants.ELEVATOR_UPPER_LIMIT_DIO);
    
        m_elevatorMotor = new WPI_TalonFX(Constants.ELEVATOR_MOTOR_ID);
        m_elevatorMotor.setNeutralMode(NeutralMode.Brake);

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

        m_sbTab.addDouble("Current Distance:", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return getMeasurement();
            };
        });
    }

    DoubleSupplier measurementSupplier = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return getMeasurement();
        };
    };

    @Override
    public void periodic() {
        super.periodic();

        if (DriverStation.isDisabled()) {
            if (isEnabled()) {
                disable();
            }
            m_elevatorMotor.set(0);
            return;
        }

        if(isElevatorUpperLimitReached()  && elevatorState == ElevatorState.raising){
            stopElevator();
        } else if(isElevatorLowerLimitReached() && elevatorState == ElevatorState.lowering){
            stopElevator();
        }
        if(isElevatorLowerLimitReached()){
            m_elevatorMotor.setSelectedSensorPosition(0);
        }
        if(isElevatorUpperLimitReached()){
            System.out.println(m_elevatorMotor.getSelectedSensorPosition());
        }


        // if(!pidController.atGoal()){
        //     int destinationEncoderValue = getElevatorEncoderValue(elevatorDestination);
        //     System.out.println("Destination: " + destinationEncoderValue + "   Current: " + m_elevatorMotor.getSelectedSensorPosition());
            
            

        //     // if(m_elevatorMotor.getSelectedSensorPosition() < destinationEncoderValue - 2000){
        //     //     raiseElevator();
        //     // } else if(m_elevatorMotor.getSelectedSensorPosition() < destinationEncoderValue + 2000){
        //     //     lowerElevator();
        //     // } else {
        //     //     currentElevatorPosition = elevatorDestination;
        //     //     elevatorDestination = ElevatorPosition.unknown;
        //     //     stopElevator();
        //     // }
        // }
        // if (pidController.atGoal()) {
        //     //System.out.println("At Goal!!!!");
        //     // disable();
        //     // m_elevatorMotor.setNeutralMode(NeutralMode.Brake);
        //     // m_elevatorMotor.set(0);
        //     // m_elevatorMotor.
        // }
    }

    private void stopElevator(){
        System.out.println("stopping elevator");
        elevatorState = ElevatorState.stopped;
        m_elevatorMotor.stopMotor();
        disable();
    }  

    // public void raiseElevator() {
    //     if(!isElevatorUpperLimitReached()){
    //         System.out.println("raising elevator");
    //         elevatorState = ElevatorState.raising;
    //         m_elevatorMotor.set(ControlMode.PercentOutput, Constants.ELEVATOR_MOTOR_SPEED);
    //     } else {
    //         if(elevatorState == ElevatorState.raising){
    //             System.out.println("Upper limit switch hit");
    //             stopElevator();
    //         }
    //     }
    // }
    
    // public void lowerElevator() {
    //     if(!isElevatorLowerLimitReached()){
    //         System.out.println("lowering elevator");
    //         elevatorState = ElevatorState.lowering;
    //         m_elevatorMotor.set(ControlMode.PercentOutput, -Constants.ELEVATOR_MOTOR_SPEED);
    //     } else {
    //         if(elevatorState == ElevatorState.lowering){
    //         System.out.println("Lower limit switch hit");
    //         stopElevator();
    //         }
    //     }
    // }  

    public void setElevatorPosition(double height){
        // elevatorDestination = position;
        // double destTicks = getElevatorEncoderValue(position);
        System.out.println("setting height: " + height);
        // currentElevatorPosition = ElevatorPosition.unknown;

        // m_elevatorMotor.setSelectedSensorPosition(0, 0, 50);
        this.m_controller.reset(getMeasurement());
        
        pidController.setGoal(height);
        enable();
    }

    // private int getElevatorEncoderValue(ElevatorPosition position){
    //     switch(position){
    //         case floor:
    //             return Constants.ELEVATOR_ENCODER_SPOT_FLOOR;
    //         case middle:
    //             return Constants.ELEVATOR_ENCODER_SPOT_MIDDLE;
    //         case top:
    //             return Constants.ELEVATOR_ENCODER_SPOT_TOP;
    //         default:
    //             System.out.println("Invalid Value WTF");
    //             return 0;
            
    //     }
    // }

    public boolean isElevatorUpperLimitReached() {
        return !m_elevatorUpLimit.get();
    }

    public boolean isElevatorLowerLimitReached() {
        return !m_elevatorDownLimit.get();
    }

    // public void goToFloor(){
    //     setElevatorPosition(ElevatorPosition.floor);
    // }

    // public void goToMiddle(){
    //     setElevatorPosition(ElevatorPosition.middle);
    // }

    // public void goToTop(){
    //     setElevatorPosition(ElevatorPosition.top);
    // }

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

        
        m_elevatorMotor.setVoltage(val);

        if (val > 0 ) {
            elevatorState = ElevatorState.raising;
        } else if (val < 0) {
            elevatorState = ElevatorState.lowering;
        } else {
            elevatorState = ElevatorState.stopped;
        }
    }

    @Override
        public double getMeasurement() {
            double ticks = m_elevatorMotor.getSelectedSensorPosition();
            double distance = ticks / 4500;
            return distance;
    }
}
