package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;

public class ParallelArmCommand extends ParallelCommandGroup {

    private ExtendoSubsystem m_extendoSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;
    private Constants.ArmDestination m_destination;

    public ParallelArmCommand(ExtendoSubsystem extendoSubsystem, ElevatorSubsystem elevatorSubsystem,
            Constants.ArmDestination destination) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_extendoSubsystem = extendoSubsystem;
        m_destination = destination;

        if (m_destination == Constants.ArmDestination.bot) {
            addCommands(
                    new InstantCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.DISTANCE_BOT)),
                    new InstantCommand(() -> m_extendoSubsystem.goToDistanceCM(ExtendoSubsystem.DISTANCE_BOT)));
        } else if (m_destination == Constants.ArmDestination.low) {
            addCommands(
                    new InstantCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.DISTANCE_LOW)),
                    new InstantCommand(() -> m_extendoSubsystem.goToDistanceCM(ExtendoSubsystem.DISTANCE_LOW)));
        } else if (m_destination == Constants.ArmDestination.middle) {
            addCommands(
                    new InstantCommand(
                            () -> m_elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.DISTANCE_MIDDLE)),
                    new InstantCommand(() -> m_extendoSubsystem.goToDistanceCM(ExtendoSubsystem.DISTANCE_MIDDLE)));
        } else if (m_destination == Constants.ArmDestination.high) {
            addCommands(
                    new InstantCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.DISTANCE_HIGH)),
                    new InstantCommand(() -> m_extendoSubsystem.goToDistanceCM(ExtendoSubsystem.DISTANCE_HIGH)));
        } else if (m_destination == Constants.ArmDestination.shelf) {
            addCommands(
                    new InstantCommand(() -> m_elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.DISTANCE_SHELF)),
                    new InstantCommand(() -> m_extendoSubsystem.goToDistanceCM(ExtendoSubsystem.DISTANCE_SHELF)));
        } else {
            System.out.println("ParallelArmCommand - Invalid ArmDestination value");
        }
    }
}
