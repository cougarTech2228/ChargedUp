package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoThreeCommand extends SequentialCommandGroup {

    public AutoThreeCommand(DrivetrainSubsystem drivetrainSubsystem, AprilTagSubsystem aprilTagSubsystem,
            XboxController controller) {

        TrajectoryConfig config = new TrajectoryConfig(
                Constants.MAX_AUTO_VELOCITY,
                Constants.MAX_AUTO_ACCELERATION)
                .setKinematics(drivetrainSubsystem.getKinematics());

        // Go out straight 18' (~5.5m) and end up turned 180 degrees to be in position
        // to pick up staged piece. Might need to add a slight y offset to line up
        // with the staged piece.
        // Trajectory pickStagedPieceTrajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         List.of(new Translation2d(2.5, 0)),
        //         new Pose2d(5.5, 0, new Rotation2d(Math.PI)),
        //         config);

        // Turn around toward grid and start your way back just enough to let
        // the DockWithAprilTag command take over
        // Trajectory turnTowardGrid = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         List.of(new Translation2d(1, 0)),
        //         new Pose2d(2.0, 0, new Rotation2d(Math.PI)),
        //         config);

        // Based on Shuffleboard seletion, create approrpiate Place Command for
        // preloaded piece
        Constants.PlacePosition preloadedPieceLevel = RobotContainer.getShuffleboardManager()
                .getStagedPieceLevel();
        Command preloadedPiecePlaceCommand;

        if ((preloadedPieceLevel == Constants.PlacePosition.HighCone) ||
                (preloadedPieceLevel == Constants.PlacePosition.MiddleCone) ||
                (preloadedPieceLevel == Constants.PlacePosition.LowCone)) {
            preloadedPiecePlaceCommand = new PlaceConeCommand(preloadedPieceLevel);
        } else {
            preloadedPiecePlaceCommand = new PlaceCubeCommand(preloadedPieceLevel);
        }

        // Based on Shuffleboard selection for staged cone placement, 0.0 for cube
        Constants.ConeOffsetPosition preloadedConeOffsetPosition = RobotContainer.getShuffleboardManager()
                .getPreloadedConeOffsetPosition();
        Command preloadedStrafeCommand;

        if ((preloadedPieceLevel == Constants.PlacePosition.HighCube) ||
                (preloadedPieceLevel == Constants.PlacePosition.MiddleCube) ||
                (preloadedPieceLevel == Constants.PlacePosition.LowCube)) {
            System.out.println("Preloaded Cube Chosen");
            preloadedStrafeCommand = new StrafeCommand(0.0);
        } else if (preloadedConeOffsetPosition == Constants.ConeOffsetPosition.Left) {
            System.out.println("Preloaded Left Cone Chosen");
            preloadedStrafeCommand = new StrafeCommand(-Constants.GRID_OFFSET_CM);
        } else {
            System.out.println("Preloaded Right Cone Chosen");
            preloadedStrafeCommand = new StrafeCommand(Constants.GRID_OFFSET_CM);
        }

        // Based on Shuffleboard seletion, create approrpiate Place Command for staged piece
        Constants.PlacePosition stagedPieceLevel = RobotContainer.getShuffleboardManager()
                .getStagedPieceLevel();
        Command stagedPiecePlaceCommand;

        if ((stagedPieceLevel == Constants.PlacePosition.HighCone) ||
                (stagedPieceLevel == Constants.PlacePosition.MiddleCone) ||
                (stagedPieceLevel == Constants.PlacePosition.LowCone)) {
            stagedPiecePlaceCommand = new PlaceConeCommand(stagedPieceLevel);
        } else {
            stagedPiecePlaceCommand = new PlaceCubeCommand(stagedPieceLevel);
        }

        // Based on Shuffleboard selection for staged cone placement, 0.0 for cube
        Constants.ConeOffsetPosition stagedConeOffsetPosition = RobotContainer.getShuffleboardManager()
                .getStagedConeOffsetPosition();
        Command stagedStrafeCommand;

        if ((stagedPieceLevel == Constants.PlacePosition.HighCube) ||
                (stagedPieceLevel == Constants.PlacePosition.MiddleCube) ||
                (stagedPieceLevel == Constants.PlacePosition.LowCube)) {
            System.out.println("Staged Cube Chosen");
            stagedStrafeCommand = new StrafeCommand(0.0);
        } else if (stagedConeOffsetPosition == Constants.ConeOffsetPosition.Left) {
            System.out.println("Staged Left Cone Chosen");
            stagedStrafeCommand = new StrafeCommand(-Constants.GRID_OFFSET_CM);
        } else {
            System.out.println("Staged Right Cone Chosen");
            stagedStrafeCommand = new StrafeCommand(Constants.GRID_OFFSET_CM);
        }

        // Determine April Tag ID
        double aprilTagId = Constants.BAD_APRIL_TAG_ID;
        if (DriverStation.getAlliance() == Alliance.Blue) {
            aprilTagId = 6.0;
        } else {
            aprilTagId = 3.0;
        }

        addCommands(new PrintCommand("Starting AutoThreeCommand"),
                preloadedStrafeCommand,
                preloadedPiecePlaceCommand,
                new InstantCommand(drivetrainSubsystem::setMotorsToCoast), // Keep from completely
                                                                           // stopping
                // new SimpleTrajectoryCommand(pickStagedPieceTrajectory, drivetrainSubsystem),
                // new SimpleTrajectoryCommand(turnTowardGrid, drivetrainSubsystem),
                new InstantCommand(drivetrainSubsystem::setMotorsToBrake),
                new DockWithAprilTagCommand(controller, drivetrainSubsystem,
                    aprilTagSubsystem, true, aprilTagId),
                stagedStrafeCommand,
                stagedPiecePlaceCommand,
                new PrintCommand("AutoThreeCommand Complete!"));
    }
}
