package frc.robot.commands;

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

        addRequirements(m_elevatorSubsystem, m_extendoSubsystem);

        addCommands(
            new SetArmHeightCommand(m_elevatorSubsystem, m_destination),
            new SetArmReachCommand(m_extendoSubsystem, m_destination));
    }
}
