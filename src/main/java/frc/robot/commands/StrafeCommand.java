package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class StrafeCommand extends CommandBase {
    private double m_distanceCM;
    private double m_speed;

    double m_currentEncoderCount;
    double m_startEncoderCount;
    double m_distanceInEncoderCounts;

    boolean m_accountForAprilTag;

    // Based on a 4" swerve wheel
    private final static double WHEEL_CIRCUMFERENCE_CM = 31.9278;

    // Falcon ticks per rotation is 2048 * SDS Mk4i Gear Ratio of 6.75:1
    private final static double TICKS_PER_ROTATION = 2048.0 * 6.75;

    /**
     * 
     * @param distance Distance in CM to drive, should always be positive
     * @param speed    speed from [-1, 1]
     *                 For Field Oriented View (FOV) mode in Teleop, negative
     *                 speed goes right, positive speed goes left.
     *                 For Robot Oriented View mode in Autonomous mode,
     *                 negative speed goes left, positive speed goes right.
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
            double offsetInCm = RobotContainer.getAprilTagManager().getTX() * 100.0;
            // System.out.println("offsetInCm: " + offsetInCm);

            // If we're strafing Right ...
            if (m_speed < 0.0) {

                // Auto is not FOV, need to invert control direction/speed
                // if (DriverStation.isAutonomous()) {
                // System.out.println("Strafing Left");
                // } else {
                // System.out.println("Strafing Right");
                // }

                // If offset is positive, subtract from constant, otherwise add to constant
                if (offsetInCm < 0.0) {
                    m_distanceCM += Math.abs(offsetInCm);
                } else {
                    m_distanceCM -= Math.abs(offsetInCm);
                }
            } else {

                // Auto is not FOV, need to invert control direction/speed
                // if (DriverStation.isAutonomous()) {
                // System.out.println("Strafing Right");
                // } else {
                // System.out.println("Strafing Left");
                // }

                // We're strafing Left ...
                // If offset is positive, add to constant, otherwise subtract from constant
                if (offsetInCm > 0.0) {
                    m_distanceCM += Math.abs(offsetInCm);
                } else {
                    m_distanceCM -= Math.abs(offsetInCm);
                }
            }
        }

        System.out.println("Strafe Command starting");

        m_distanceInEncoderCounts = ((m_distanceCM / WHEEL_CIRCUMFERENCE_CM) * TICKS_PER_ROTATION);

        m_currentEncoderCount = RobotContainer.getDrivetrainSubsystem().getEncoderCount();
        m_startEncoderCount = m_currentEncoderCount;
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
        return ((m_currentEncoderCount <= (m_startEncoderCount - m_distanceInEncoderCounts)) ||
                (m_currentEncoderCount >= (m_startEncoderCount + m_distanceInEncoderCounts)));
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.getDrivetrainSubsystem().stopMotors();
        RobotContainer.getDrivetrainSubsystem().setMotorsToBrake();
        System.out.println("StrafeCommand finished");
    }
}