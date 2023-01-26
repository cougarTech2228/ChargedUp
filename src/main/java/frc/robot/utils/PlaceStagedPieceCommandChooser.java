package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.SetArmHeightCommand;
import frc.robot.commands.SetArmReachCommand;

public class PlaceStagedPieceCommandChooser {

    public PlaceStagedPieceCommandChooser() {

    }

    public SequentialCommandGroup getPlaceStagedPieceCommand() {

        // Based on Shuffleboard selection, create appropriate Place Command for
        // staged piece
        Constants.PlacePosition stagedPieceLevel = RobotContainer.getShuffleboardManager()
                .getStagedPieceLevel();

        if (stagedPieceLevel == Constants.PlacePosition.HighCone) {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(new SetArmHeightCommand(Constants.ARM_HIGH_CONE_HEIGHT_CM),
                            new SetArmReachCommand(Constants.ARM_HIGH_CONE_REACH_CM)),
                    new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true)));

        } else if (stagedPieceLevel == Constants.PlacePosition.MiddleCone) {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(new SetArmHeightCommand(Constants.ARM_MIDDLE_CONE_HEIGHT_CM),
                            new SetArmReachCommand(Constants.ARM_MIDDLE_CONE_REACH_CM)),
                    new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true)));

        } else if (stagedPieceLevel == Constants.PlacePosition.LowCone) {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(new SetArmHeightCommand(Constants.ARM_LOW_CONE_HEIGHT_CM),
                            new SetArmReachCommand(Constants.ARM_LOW_CONE_REACH_CM)),
                    new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true)));

        } else if (stagedPieceLevel == Constants.PlacePosition.HighCube) {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(new SetArmHeightCommand(Constants.ARM_HIGH_CUBE_HEIGHT_CM),
                            new SetArmReachCommand(Constants.ARM_HIGH_CUBE_REACH_CM)),
                    new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true)));

        } else if (stagedPieceLevel == Constants.PlacePosition.MiddleCube) {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(new SetArmHeightCommand(Constants.ARM_MIDDLE_CUBE_HEIGHT_CM),
                            new SetArmReachCommand(Constants.ARM_MIDDLE_CUBE_REACH_CM)),
                    new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true)));

        } else if (stagedPieceLevel == Constants.PlacePosition.LowCube) {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(new SetArmHeightCommand(Constants.ARM_LOW_CUBE_HEIGHT_CM),
                            new SetArmReachCommand(Constants.ARM_LOW_CUBE_REACH_CM)),
                    new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true)));

        } else {
            System.out.println("Error selecting place position");
        }

        return null;
    }
}
