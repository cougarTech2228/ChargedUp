package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ParallelArmCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class PlacePieceCommandChooser {

    Constants.PlacePosition m_pieceLevel;
    private static ElevatorSubsystem m_elevatorSubsystem;
    private static ExtendoSubsystem m_extendoSubsystem;
    private static PneumaticSubsystem m_pneumaticSubsystem;

    public PlacePieceCommandChooser(ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem,
            PneumaticSubsystem pneumaticSubsystem, Constants.PlacePosition pieceLevel) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_extendoSubsystem = extendoSubsystem;
        m_pneumaticSubsystem = pneumaticSubsystem;
        m_pieceLevel = pieceLevel;
    }

    public SequentialCommandGroup getPlacePieceCommand() {

        if (m_pieceLevel == Constants.PlacePosition.HighCone || m_pieceLevel == Constants.PlacePosition.HighCube) {
            return new SequentialCommandGroup(
                    new InstantCommand(
                            () -> new ParallelArmCommand(m_extendoSubsystem, m_elevatorSubsystem, Constants.ArmDestination.high)),
                    new InstantCommand(() -> m_pneumaticSubsystem.openGripper()));

        } else if (m_pieceLevel == Constants.PlacePosition.MiddleCone
                || m_pieceLevel == Constants.PlacePosition.MiddleCube) {
            return new SequentialCommandGroup(
                    new InstantCommand(
                            () -> new ParallelArmCommand(m_extendoSubsystem, m_elevatorSubsystem, Constants.ArmDestination.middle)),
                    new InstantCommand(() -> m_pneumaticSubsystem.openGripper()));

        } else if (m_pieceLevel == Constants.PlacePosition.LowCone || m_pieceLevel == Constants.PlacePosition.LowCube) {
            return new SequentialCommandGroup(
                    new InstantCommand(
                            () -> new ParallelArmCommand(m_extendoSubsystem, m_elevatorSubsystem, Constants.ArmDestination.low)),
                    new InstantCommand(() -> m_pneumaticSubsystem.openGripper()));

        } else {
            System.out.println("Error selecting place position");
        }

        return null;
    }
}
