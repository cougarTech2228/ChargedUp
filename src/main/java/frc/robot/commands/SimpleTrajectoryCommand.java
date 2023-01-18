package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class SimpleTrajectoryCommand extends SequentialCommandGroup {

    private static final double kMaxAngularSpeedRadiansPerSecond = Math.PI / 2.0;
    private static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI / 4.0;

    private static final double kPXController = 0.6;
    private static final double kPYController = 0.6;
    private static final double kPThetaController = 0.6;

    /* Constraint for the motion profilied robot angle controller */
    private static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public SimpleTrajectoryCommand(Trajectory trajectory, DrivetrainSubsystem drivetrainSubsystem) {

        var thetaController = new ProfiledPIDController(
                kPThetaController, 0, 0, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                drivetrainSubsystem::getPose,
                drivetrainSubsystem.getKinematics(),
                new PIDController(kPXController, 0, 0),
                new PIDController(kPYController, 0, 0),
                thetaController,
                drivetrainSubsystem::setModuleStates,
                drivetrainSubsystem);

        addCommands(
                new PrintCommand("Starting SimpleTrajectoryCommand"),
                new InstantCommand(() -> drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new PrintCommand("Ending SimpleTrajectoryCommand"));
    }
}