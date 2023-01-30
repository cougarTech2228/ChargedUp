package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class StrafeCommand extends CommandBase {
    private double m_distanceCM;
    private double m_speed;
    private boolean m_isDone;

    double m_currentEncoderCount;
    double m_lastEncoderCount;
    double m_startEncoderCount;
    double m_distanceInEncoderCounts;
    int m_robotNotMovingCount;

    boolean m_accountForAprilTag;

    // Based on a 4" swerve wheel
    private final static double WHEEL_CIRCUMFERENCE_CM = 31.9278;

    // Falcon ticks per rotation is 2048 * SDS Mk4i Gear Ratio of 6.75:1
    private final static double TICKS_PER_ROTATION = 2048.0 * 6.75;

    /**
     * 
     * @param distance Distance in CM to drive, should always be positive
     * @param speed    speed from [-1, 1]
     *                 negative speed goes right (FOV), positive speed goes left
     *                 (FOV)
     * @throws Exception
     */
    public StrafeCommand(double distanceCM, double speed, boolean accountForAprilTag) {

        m_distanceCM = Math.abs(distanceCM);
        m_speed = speed;
        m_accountForAprilTag = accountForAprilTag;

        if (m_speed < -1.0 || m_speed > 1.0) {
            System.out.println("ERROR: Speed value passed into StrafeCommand out of [-1, 1]");
            m_speed = 0.0;
        }

        addRequirements(RobotContainer.getDrivetrainSubsystem());
    }

    @Override
    public void initialize() {

        // If we're strafing away from an April Tag and it's not a manual strafe from
        // the Button Board's joystick, use the last recorded Tx value from the April
        // Tag detection to make a better estimate on how far we need to strafe in
        // order to line up with the Cone Nodes.
        if (m_accountForAprilTag) {
            m_distanceCM = RobotContainer.getAprilTagManager().getStrafeOffsetGrid(m_distanceCM);
        }

        System.out.println("Strafe Command starting");
        m_isDone = false;

        m_distanceInEncoderCounts = ((m_distanceCM / WHEEL_CIRCUMFERENCE_CM) * TICKS_PER_ROTATION);

        m_currentEncoderCount = RobotContainer.getDrivetrainSubsystem().getEncoderCount();
        m_startEncoderCount = m_currentEncoderCount;
        m_lastEncoderCount = m_currentEncoderCount;
        m_robotNotMovingCount = 0;

        // System.out.println("Distance in encoder count to travel: " +
        // m_distanceInEncoderCounts);
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

        // Checks for both encoder count directions
        if ((m_currentEncoderCount <= (m_startEncoderCount - m_distanceInEncoderCounts)) ||
                (m_currentEncoderCount >= (m_startEncoderCount + m_distanceInEncoderCounts))) {
            RobotContainer.getDrivetrainSubsystem().stopMotors();
            RobotContainer.getDrivetrainSubsystem().setMotorsToBrake();
            System.out.println("StrafeCommand finished");
            m_isDone = true;
        }

        // If we've started moving but then stop moving due to some unforseen issue
        // like being blocked by another robot or field element, we need to end this
        // command.
        if ((m_startEncoderCount != m_currentEncoderCount) &&
                (m_lastEncoderCount == m_currentEncoderCount)) {

            // Because of the CAN bus utilization, we've turned down the frequency
            // at which encoder counts are reported for the Falcon 500s. We need
            // to wait a few periodic cycles to make sure that the robot has really
            // stopped moving.
            if (m_robotNotMovingCount++ > 2) {
                System.out.println("Robot is obstructed...StrafeCommand finished");
                m_robotNotMovingCount = 0;
                m_isDone = true;
            }
        } else {
            m_lastEncoderCount = m_currentEncoderCount;
        }

        return m_isDone;
    }
}