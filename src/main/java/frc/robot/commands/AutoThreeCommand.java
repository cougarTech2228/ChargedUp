package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoThreeCommand extends SequentialCommandGroup {

    public AutoThreeCommand() {

        // TODO - do we want to do something cool at each stage like with LEDs?
        // We could create multiple eventMaps
        HashMap<String, Command> eventMap = new HashMap<>();

        addCommands(new PrintCommand("Starting AutoThreeCommand"),
                new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setPathPlannerDriving),
                new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setMotorsToBrake),
                new PlacePreloadedPieceCommand(),
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto3_out", eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto3_back", eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setNotPathPlannerDriving),
                new DockWithAprilTagCommand(false),
                new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setPathPlannerDriving),
                new PlaceStagedPieceCommand(),
                new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setNotPathPlannerDriving),
                new PrintCommand("AutoThreeCommand Complete!"));
    }
}
