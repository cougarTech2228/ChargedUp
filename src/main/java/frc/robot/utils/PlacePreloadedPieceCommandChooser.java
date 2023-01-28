package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.SetElevatorHeightCommand;
import frc.robot.commands.SetArmReachCommand;

public class PlacePreloadedPieceCommandChooser {

    public PlacePreloadedPieceCommandChooser() {

    }

    public SequentialCommandGroup getPlacePreloadedPieceCommand() {

        // Based on Shuffleboard selection, create appropriate Place Command for
        // preloaded piece
        Constants.PlacePosition preloadedPieceLevel = RobotContainer.getShuffleboardSubsystem()
                .getPreloadedPieceLevel();

        if (preloadedPieceLevel == Constants.PlacePosition.HighCone) {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(new SetElevatorHeightCommand(Constants.ARM_HIGH_CONE_HEIGHT_CM),
                            new SetArmReachCommand(Constants.ARM_HIGH_CONE_REACH_CM)),
                    new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true)));

        } else if (preloadedPieceLevel == Constants.PlacePosition.MiddleCone) {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(new SetElevatorHeightCommand(Constants.ARM_MIDDLE_CONE_HEIGHT_CM),
                            new SetArmReachCommand(Constants.ARM_MIDDLE_CONE_REACH_CM)),
                    new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true)));

        } else if (preloadedPieceLevel == Constants.PlacePosition.LowCone) {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(new SetElevatorHeightCommand(Constants.ARM_LOW_CONE_HEIGHT_CM),
                            new SetArmReachCommand(Constants.ARM_LOW_CONE_REACH_CM)),
                    new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true)));

        } else if (preloadedPieceLevel == Constants.PlacePosition.HighCube) {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(new SetElevatorHeightCommand(Constants.ARM_HIGH_CUBE_HEIGHT_CM),
                            new SetArmReachCommand(Constants.ARM_HIGH_CUBE_REACH_CM)),
                    new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true)));

        } else if (preloadedPieceLevel == Constants.PlacePosition.MiddleCube) {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(new SetElevatorHeightCommand(Constants.ARM_MIDDLE_CUBE_HEIGHT_CM),
                            new SetArmReachCommand(Constants.ARM_MIDDLE_CUBE_REACH_CM)),
                    new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true)));

        } else if (preloadedPieceLevel == Constants.PlacePosition.LowCube) {
            return new SequentialCommandGroup(
                    new ParallelCommandGroup(new SetElevatorHeightCommand(Constants.ARM_LOW_CUBE_HEIGHT_CM),
                            new SetArmReachCommand(Constants.ARM_LOW_CUBE_REACH_CM)),
                    new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true)));

        } else {
            System.out.println("Error selecting place position");
        }

        return null;
    }
}
