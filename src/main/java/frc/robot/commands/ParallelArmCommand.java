package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;

public class ParallelArmCommand extends ParallelCommandGroup {

    private ExtendoSubsystem m_extendoSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;
    private ArmDestination m_destination;

    public enum ArmDestination {
        high,
        mid,
        low,
        bot
    }

    public ParallelArmCommand(ExtendoSubsystem extendoSubsystem, ElevatorSubsystem elevatorSubsystem,
    ArmDestination destination) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_extendoSubsystem = extendoSubsystem;
        m_destination = destination;

        if (m_destination == ArmDestination.bot) {
            addCommands(new InstantCommand(() ->m_elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.DISTANCE_BOT)),
            new InstantCommand(() ->m_extendoSubsystem.goToDistanceCM(ExtendoSubsystem.DISTANCE_BOT)));
        }
    }
}
