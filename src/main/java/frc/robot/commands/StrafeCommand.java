package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class StrafeCommand extends CommandBase {
    private double m_distanceCM;
    private double m_speed;
    private boolean m_isDone = false;

    double m_currentEncoderCount;
    double m_endCount;

    private final static double WHEEL_CIRCUMFERENCE_CM = 31.9278;
    private final static double TICKS_PER_ROTATION = 2048.0;

    /**
     * 
     * @param distance Distance in CM to drive, positive values drive right,
     *                 negative values drive left
     * @param speed    speed from 0 -> 1
     * @throws Exception
     */
    public StrafeCommand(double distanceCM, double speed) {

        m_distanceCM = distanceCM;
        m_speed = speed;

        if (m_speed < 0 || m_speed > 1) {
            m_speed = 0;
        }

        //addRequirements(RobotContainer.getDrivetrainSubsystem());
    }

    @Override
    public void initialize() {
        System.out.println("Strafe Command starting");
        m_isDone = false;
        m_currentEncoderCount = RobotContainer.getDrivetrainSubsystem().getEncoderCount();

        if (m_distanceCM > 0) {
            m_endCount = m_currentEncoderCount + ((m_distanceCM / WHEEL_CIRCUMFERENCE_CM) * TICKS_PER_ROTATION);
        } else {
            m_endCount = m_currentEncoderCount - ((-m_distanceCM / WHEEL_CIRCUMFERENCE_CM) * TICKS_PER_ROTATION);
            m_speed = -m_speed;
        }
    }

    @Override
    public void execute() {

        RobotContainer.getDrivetrainSubsystem().drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.0,
                m_speed * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                0.0,
                RobotContainer.getDrivetrainSubsystem().getGyroscopeRotation()));

        m_currentEncoderCount = RobotContainer.getDrivetrainSubsystem().getEncoderCount();
    }

    @Override
    public boolean isFinished() {
        if (m_distanceCM > 0) {
            if (m_currentEncoderCount >= m_endCount) {
                RobotContainer.getDrivetrainSubsystem().stopMotors();
                RobotContainer.getDrivetrainSubsystem().setMotorsToBrake();
                System.out.println("StrafeCommand finished");
                m_isDone = true;
            }
        } else {
            if (m_currentEncoderCount <= m_endCount) {
                RobotContainer.getDrivetrainSubsystem().stopMotors();
                RobotContainer.getDrivetrainSubsystem().setMotorsToBrake();
                System.out.println("StrafeCommand finished");
                m_isDone = true;
            }
        }

        return m_isDone;
    }
}