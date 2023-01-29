package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.OutPathFileNameChooser;
import frc.robot.utils.PlacePreloadedPieceCommandChooser;
import frc.robot.utils.PlaceStagedPieceCommandChooser;

public class AutoThreeCommand extends SequentialCommandGroup {

    private enum CommandSelector {

        STRAFE_LEFT,
        STRAFE_RIGHT,
        STRAFE_NONE
    }

    private double m_startTime = 0;

    public AutoThreeCommand() {

        // TODO - do we want to do something cool at each stage like with LEDs?
        // We could create multiple eventMaps
        HashMap<String, Command> eventMap = new HashMap<>();

        // Select the "out path" file based on Shuffleboard configuration
        OutPathFileNameChooser outPathFileNameChooser = new OutPathFileNameChooser();
        String outPathFileName = outPathFileNameChooser.getOutPathFileName();

        PlacePreloadedPieceCommandChooser placePreloadedPieceCommandChooser = new PlacePreloadedPieceCommandChooser();
        SequentialCommandGroup placePreloadedPieceSequentialCommandGroup = placePreloadedPieceCommandChooser
                .getPlacePreloadedPieceCommand();

        PlaceStagedPieceCommandChooser placeStagedPieceCommandChooser = new PlaceStagedPieceCommandChooser();
        SequentialCommandGroup placeStagedPieceSequentialCommandGroup = placeStagedPieceCommandChooser
                .getPlaceStagedPieceCommand();

        addCommands(new InstantCommand(() -> printStartCommand()),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().zeroGyroscope()),
                new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setMotorsToBrake),
                placePreloadedPieceSequentialCommandGroup,
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(true)),
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), outPathFileName, eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto3_back", eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new DockWithAprilTagCommand(false, true),
                new SelectCommand(
                        Map.ofEntries(
                                Map.entry(CommandSelector.STRAFE_LEFT,
                                        new StrafeCommand(Constants.GRID_STRAFE_DISTANCE, -Constants.STRAFE_SPEED,
                                                true)),
                                Map.entry(CommandSelector.STRAFE_RIGHT,
                                        new StrafeCommand(Constants.GRID_STRAFE_DISTANCE, Constants.STRAFE_SPEED,
                                                true)),
                                Map.entry(CommandSelector.STRAFE_NONE,
                                        new PrintCommand("We're already lined up, no strafing necessary"))),
                        this::selectStagedStrafe),
                placeStagedPieceSequentialCommandGroup,
                new InstantCommand(() -> printEndCommand()));
    }

    // Choose whether or not we have to strafe and in what direction based on
    // Shuffleboard inputs
    private CommandSelector selectStagedStrafe() {

        Constants.PlacePosition placePosition = RobotContainer.getShuffleboardSubsystem().getStagedPieceLevel();
        Constants.ConeOffsetPosition conePosition = RobotContainer.getShuffleboardSubsystem()
                .getStagedConeOffsetPosition();

        if ((placePosition == Constants.PlacePosition.HighCone) ||
                (placePosition == Constants.PlacePosition.MiddleCone) ||
                (placePosition == Constants.PlacePosition.LowCone)) {
            if (conePosition == Constants.ConeOffsetPosition.Left) {
                return CommandSelector.STRAFE_LEFT;
            } else {
                return CommandSelector.STRAFE_RIGHT;
            }
        } else {
            return CommandSelector.STRAFE_NONE;
        }
    }

    private void printStartCommand() {
        m_startTime = Timer.getFPGATimestamp();
        System.out.println("Starting AutoThreeCommand");
    }

    private void printEndCommand() {
        System.out.println("AutoThreeCommand completed in " + (Timer.getFPGATimestamp() - m_startTime) + " seconds");
    }
}
