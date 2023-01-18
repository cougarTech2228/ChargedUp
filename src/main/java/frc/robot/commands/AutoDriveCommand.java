package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveCommand extends CommandBase {

    private  DrivetrainSubsystem m_drivetrainSubsystem;
    private double m_xChange;
    private double m_yChange;
    private double m_rotationChange;
    private PPSwerveControllerCommand m_command;

    public PathPlannerTrajectory getTraj() {
        Pose2d currentPose = m_drivetrainSubsystem.getOdometry().getPoseMeters();

        Translation2d currentPosition = new Translation2d(currentPose.getX(), currentPose.getY());
        Translation2d newPopsition = new Translation2d(currentPose.getX() + m_xChange, currentPose.getY() + m_yChange);

        double startHeading = 1/Math.tan((newPopsition.getY() - currentPosition.getY()) / (newPopsition.getX() - currentPosition.getX()));
        System.out.println("startHeading:  " + startHeading);
        
        Rotation2d newRotation = currentPose.getRotation().plus(Rotation2d.fromDegrees(m_rotationChange));

        System.out.println("HERE@@@@@@@@@@");
        System.out.println("Current: " + currentPose.toString() + 
                           " New : " + newRotation);
        PathPlannerTrajectory traj1 = PathPlanner.generatePath(
           //PathPoint(Translation2d position, Rotation2d heading, Rotation2d holonomicRotation)
            new PathConstraints(4, 3), 
            new PathPoint(
                new Translation2d(currentPose.getX(), currentPose.getY()), currentPose.getRotation(),
                 currentPose.getRotation()), // position, heading
            new PathPoint(new Translation2d(currentPose.getX() + m_xChange, currentPose.getY() + m_yChange),  newRotation, newRotation) // position, heading
        );
        

        return traj1;
    }

    public AutoDriveCommand(DrivetrainSubsystem drivetrainSubsystem, double xChange, double yChange, double rotationChange){
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_xChange = xChange;
        m_yChange = yChange;
        m_rotationChange = rotationChange;


    }

    @Override
    public final void initialize() {
        m_command = new PPSwerveControllerCommand(
            getTraj(),
            m_drivetrainSubsystem::getPose, // Pose supplier
            m_drivetrainSubsystem.getKinematics(), // SwerveDriveKinematics
                new PIDController(0.1, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will
                                              // only use feedforwards.
                new PIDController(0.12, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(2, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0
                                            // will only use feedforwards.
                m_drivetrainSubsystem::setModuleStates, // Module states consumer
                m_drivetrainSubsystem // Requires the drive subsystem
        );

        m_command.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_command.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_command.end(interrupted);

        Pose2d currentPose = m_drivetrainSubsystem.getOdometry().getPoseMeters();
        System.out.println("Pose at end: " + currentPose.toString());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_command.isFinished();
    }
}
