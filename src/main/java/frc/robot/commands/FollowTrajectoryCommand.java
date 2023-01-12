
package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FollowTrajectoryCommand extends SequentialCommandGroup {

    public FollowTrajectoryCommand(DrivetrainSubsystem drivetrain, PathPlannerTrajectory traj, boolean isFirstPath) {

        addCommands (
            new InstantCommand(() -> {
                // Reset odometry for the first path you run during auto
                if(isFirstPath) {
                    drivetrain.getOdometry().resetPosition(Rotation2d.fromDegrees(drivetrain.getYaw()), drivetrain.getSwerveModulePositions() ,traj.getInitialHolonomicPose());
                }
            }),
            new PPSwerveControllerCommand(
                traj, 
                drivetrain::getPose, // Pose supplier
                drivetrain.getKinematics(), // SwerveDriveKinematics
                new PIDController(0.1, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0.12, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(2, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                drivetrain::setModuleStates, // Module states consumer
                drivetrain // Requires the drive subsystem
            )
        );
    }
}

