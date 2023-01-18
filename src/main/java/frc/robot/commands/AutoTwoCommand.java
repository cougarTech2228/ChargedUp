package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoTwoCommand extends SequentialCommandGroup {

    public AutoTwoCommand() {

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
        Command preloadedStrafeCommand = new PrintCommand("TODO preloadedStrafeCommand");

        // if ((preloadedPieceLevel == Constants.PlacePosition.HighCube) ||
        //         (preloadedPieceLevel == Constants.PlacePosition.MiddleCube) ||
        //         (preloadedPieceLevel == Constants.PlacePosition.LowCube)) {
        //     System.out.println("Preloaded Cube Chosen");
        //     preloadedStrafeCommand = new PrintCommand("Already in position");
        // } else if (preloadedConeOffsetPosition == Constants.ConeOffsetPosition.Left) {
        //     System.out.println("Preloaded Left Cone Chosen");
        //     preloadedStrafeCommand = new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(),
        //             "grid_strafe_left", eventMap,
        //             Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
        // } else {
        //     System.out.println("Preloaded Right Cone Chosen");
        //     preloadedStrafeCommand = new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(),
        //             "grid_strafe_right", eventMap,
        //             Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
        // }

        addCommands(new PrintCommand("Starting AutoTwoCommand"),
                new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setPathPlannerDriving),
                new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setMotorsToBrake),
                new ParallelCommandGroup(
                        preloadedStrafeCommand,
                        preloadedPiecePlaceCommand),
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto2_out", eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto2_back", eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setNotPathPlannerDriving),
                new PrintCommand("AutoTwoCommand Complete!"));
    }
}
