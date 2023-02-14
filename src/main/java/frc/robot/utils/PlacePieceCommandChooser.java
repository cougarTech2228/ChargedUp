package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;

public class PlacePieceCommandChooser {

    Constants.PlacePosition m_pieceLevel;
    private static ElevatorSubsystem m_elevatorSubsystem;
    private static ExtendoSubsystem m_extendoSubsystem;

    public PlacePieceCommandChooser(ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem, Constants.PlacePosition pieceLevel) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_extendoSubsystem = extendoSubsystem;
        m_pieceLevel = pieceLevel;
    }

    public SequentialCommandGroup getPlacePieceCommand() {

        // Based on Shuffleboard selection, create appropriate Place Command for
        // preloaded piece
        // Constants.PlacePosition preloadedPieceLevel = RobotContainer.getShuffleboardSubsystem()
        //         .getPreloadedPieceLevel();

        if (m_pieceLevel == Constants.PlacePosition.HighCone || m_pieceLevel == Constants.PlacePosition.HighCube) {
            return new SequentialCommandGroup(
                new InstantCommand(() -> new ArmCommand(m_extendoSubsystem, m_elevatorSubsystem, ArmCommand.Destination.high)),
                new InstantCommand(() -> RobotContainer.getPneumaticSubsystem().openGripper()));

        } else if (m_pieceLevel == Constants.PlacePosition.MiddleCone || m_pieceLevel == Constants.PlacePosition.MiddleCube) {
            return new SequentialCommandGroup(
                new InstantCommand(() -> new ArmCommand(m_extendoSubsystem, m_elevatorSubsystem, ArmCommand.Destination.mid)),
                new InstantCommand(() -> RobotContainer.getPneumaticSubsystem().openGripper()));

        } else if (m_pieceLevel == Constants.PlacePosition.LowCone || m_pieceLevel == Constants.PlacePosition.LowCube) {
            return new SequentialCommandGroup(
                new InstantCommand(() -> new ArmCommand(m_extendoSubsystem, m_elevatorSubsystem, ArmCommand.Destination.low)),
                new InstantCommand(() -> RobotContainer.getPneumaticSubsystem().openGripper()));

        } else {
            System.out.println("Error selecting place position");
        }

        return null;
    }
}
