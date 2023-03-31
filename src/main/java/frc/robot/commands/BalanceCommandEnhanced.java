package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;

import static frc.robot.Constants.DRIVETRAIN_WHEEL_CIRCUMFERENCE_CM;
import static frc.robot.Constants.DRIVETRAIN_TICKS_PER_ROTATION;

public class BalanceCommandEnhanced extends CommandBase{
    private DrivetrainSubsystem m_drivetrainSubsystem;
    private LEDStripSubsystem m_ledStripSubsystem;

    private double m_encoderCountStart;

    private static final double GOING_OUT_FLAT_SPEED = -.3;
    private static final double GOING_OUT_CLIMB_SPEED = -.3;
    private static final double GOING_OUT_DECENDING_SPEED = -.3;
    private static final double GOING_OUT_EXITING_COMMUNITY_SPEED = -.3;
    private static final double EXITING_COMMUNITY_DISTANCE_CM = 75;
    private static final double EXITING_COMMUNITY_ENCODER_DISTANCE = 
        ((EXITING_COMMUNITY_DISTANCE_CM / DRIVETRAIN_WHEEL_CIRCUMFERENCE_CM) * DRIVETRAIN_TICKS_PER_ROTATION);
    private static final double GOING_BACK_FLAT_SPEED = .3;
    private static final double RAMP_ANGLE = 8;
    private static final double RAMP_ANGLE_DESC = 6;

    private final double kP = 0.10;
    private final double kI = 0.00;
    private final double kD = 0.0;

    private final PIDController m_pidController;
    private int m_engaged_count;

    private boolean m_goOutAndBack;

    private enum state {
        IDLE,
        GOING_OUT_FLAT,
        GOING_OUT_CLIMB,
        GOING_OUT_DECENDING,
        GOING_OUT_EXITING_COMMUNITY,
        GOING_BACK_FLAT,
        AUTO_ENGAGE,
        ENGAGED
    }

    // private state m_currentState = state.IDLE;
    private state m_currentState = state.GOING_BACK_FLAT;
    
    
    public BalanceCommandEnhanced(boolean goOutAndBack, DrivetrainSubsystem drivetrain, LEDStripSubsystem ledStripSubsystem){
        m_drivetrainSubsystem = drivetrain;
        m_ledStripSubsystem = ledStripSubsystem;
        addRequirements(m_drivetrainSubsystem);
        m_goOutAndBack = goOutAndBack;

        m_pidController = new PIDController(kP, kI, kD);
        m_pidController.setTolerance(3.5);
    }

    @Override
    public void initialize() {
        System.out.println("BalanceCommand Command starting");
        m_drivetrainSubsystem.setPathPlannerDriving(false);
        m_currentState = state.IDLE;
    }

    private void stateTransition(state newState) {
        System.out.println("Balance State " + m_currentState + " --> " + newState);
        m_currentState = newState;
    }

    /**
     * Drive forwards or backwards at a given speed
     * @param speed negative is towards the center of the field, positive is towards the driver's station
     */
    private void drive(double speed) {
        
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            speed * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            0.0,
            0.0,
            m_drivetrainSubsystem.getGyroscopeRotation()));
    }

    private void enterAutoEngage() {
        stateTransition(state.AUTO_ENGAGE);
        m_pidController.setSetpoint(0);
        m_pidController.reset();
        m_engaged_count = 0;
        //System.out.println("roll: " + m_drivetrainSubsystem.getRoll());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (m_currentState){
            case IDLE:
                /// start of the match, bot is assumed to be lined up with the grid after placing a cone
                stateTransition(state.GOING_OUT_FLAT);
                break;

            case GOING_OUT_FLAT:
                /// We have started the auto routine, and are currently flat on the floor, driving
                /// backwards into the charging station
                if (m_drivetrainSubsystem.getRoll() > RAMP_ANGLE) {
                    if (m_goOutAndBack) {
                        stateTransition(state.GOING_OUT_CLIMB);
                    } else {
                        enterAutoEngage();
                    }
                } else {
                    drive(GOING_OUT_FLAT_SPEED);
                }
                break;

            case GOING_OUT_CLIMB:
                /// We have started to climb the charging station heading towards the middle of the field
                /// we will stay in this state until we start to descend
                System.out.println("roll: " + m_drivetrainSubsystem.getRoll());
                if (m_drivetrainSubsystem.getRoll() < -RAMP_ANGLE_DESC) {
                    stateTransition(state.GOING_OUT_DECENDING);
                    // System.out.println("roll: " + m_drivetrainSubsystem.getRoll());
                } else {               
                    drive(GOING_OUT_CLIMB_SPEED);
                }
                break;

            case GOING_OUT_DECENDING:
                /// We are now driving off the charging station. We need to keep driving until we are flat on the ground
                if (Math.abs(m_drivetrainSubsystem.getRoll()) < 3) {
                    stateTransition(state.GOING_OUT_EXITING_COMMUNITY);
                    // System.out.println("roll: " + m_drivetrainSubsystem.getRoll());
                    m_encoderCountStart = m_drivetrainSubsystem.getEncoderCount();
                } else {
                    drive(GOING_OUT_DECENDING_SPEED);
                }
                break;

            case GOING_OUT_EXITING_COMMUNITY:
                /// We are off the charging station, and should drive out a little bit more to clear the station
                double currentEncoderCount = m_drivetrainSubsystem.getEncoderCount();

                if (Math.abs(currentEncoderCount - m_encoderCountStart) > EXITING_COMMUNITY_ENCODER_DISTANCE) {
                    stateTransition(state.GOING_BACK_FLAT);
                    System.out.println( "encoder count: currentEncoderCount: " +
                        currentEncoderCount + ", " + m_encoderCountStart);
                } else {
                    drive(GOING_OUT_EXITING_COMMUNITY_SPEED);
                }
                
                break;

            case GOING_BACK_FLAT:
                /// We are heading back towards the charging station to attempt to dock and engage
                if (m_drivetrainSubsystem.getRoll() < -RAMP_ANGLE) {
                    enterAutoEngage();
                } else {
                    drive(GOING_BACK_FLAT_SPEED);
                }
                break;

            case AUTO_ENGAGE:
                /// In this state, we will continue to read the IMU, and adjust our position until we are balanced

                /*
                *    <<--Bot-front
                *     <<<  +speed     -speed >>>
                *   roll > 0          roll < 0
                *          ____________
                *       _-'            '-_
                *    _-'                  '-_
                */

                double voltage = m_pidController.calculate(m_drivetrainSubsystem.getRoll());
                voltage = MathUtil.clamp(voltage, -4, 4);
                double voltage_ff = 0.3;

                if (voltage > 0) {
                    voltage += voltage_ff;
                } else if (voltage < 1) {
                    //voltage -= voltage_ff;
                    voltage = -1.175;
                }

                System.out.println( "Auto engage PID voltage: " + voltage);

                m_drivetrainSubsystem.driveRaw(voltage, voltage, voltage, voltage, 0, 0, 0, 0);
                if (m_pidController.atSetpoint()){
                    System.out.println("At goal! : " + m_engaged_count);
                    // 20ms * 50 times says we're properly engaged
                    if (m_engaged_count++ > 20) {
                        /* We are fully level (and have been for a period of time)
                            set the wheels to lock position:
                                /   \
                                \   /
                        */
                        m_drivetrainSubsystem.lockWheels();
                        stateTransition(state.ENGAGED);
                        m_ledStripSubsystem.autoPretty();
                    }
                } else {
                    // System.out.println("roll: " + m_drivetrainSubsystem.getRoll());
                    m_engaged_count = 0;
                }
                break;

            case ENGAGED:
                // we're done.
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("BalanceCommand finished");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (m_currentState == state.ENGAGED);
    }
}
