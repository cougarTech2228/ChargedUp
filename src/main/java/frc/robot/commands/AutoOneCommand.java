package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoOneCommand extends SequentialCommandGroup {

    public AutoOneCommand() {

        // TODO - do we want to do something cool at each stage like with LEDs?
        // We could create multiple eventMaps
        HashMap<String, Command> eventMap = new HashMap<>();

        // Based on Shuffleboard seletion, create appropriate Place Command for
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
            preloadedStrafeCommand = new PrintCommand("Already in position");
        } else if (preloadedConeOffsetPosition == Constants.ConeOffsetPosition.Left) {
            System.out.println("Preloaded Left Cone Chosen");
            preloadedStrafeCommand = new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(),
                    "grid_strafe_left", eventMap,
                    Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
        } else {
            System.out.println("Preloaded Right Cone Chosen");
            preloadedStrafeCommand = new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(),
                    "grid_strafe_right", eventMap,
                    Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
        }

        // Based on Shuffleboard seletion, create appropriate Place Command for staged
        // piece
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
            stagedStrafeCommand = new PrintCommand("Already in position");
        } else if (stagedConeOffsetPosition == Constants.ConeOffsetPosition.Left) {
            System.out.println("Staged Left Cone Chosen");
            stagedStrafeCommand = new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(),
                    "grid_strafe_left", eventMap,
                    Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
        } else {
            System.out.println("Staged Right Cone Chosen");
            stagedStrafeCommand = new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(),
                    "grid_strafe_right", eventMap,
                    Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
        }

        addCommands(new PrintCommand("Starting AutoOneCommand"),
                new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setPathPlannerDriving),
                new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setMotorsToBrake),
                new ParallelCommandGroup(
                        preloadedStrafeCommand,
                        preloadedPiecePlaceCommand),
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto1_out", eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto1_back", eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new DockWithAprilTagCommand(true),
                new ParallelCommandGroup(
                        stagedStrafeCommand,
                        stagedPiecePlaceCommand),
                new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setNotPathPlannerDriving),
                new PrintCommand("AutoOneCommand Complete!"));
    }
}
