package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetArmHeightCommand extends CommandBase {

    private ElevatorSubsystem m_elevatorSubsystem;
    private Constants.ArmDestination m_destination;

    public SetArmHeightCommand(ElevatorSubsystem elevatorSubsystem,
            Constants.ArmDestination destination) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_destination = destination;
    }

    @Override
    public void initialize() {
        System.out.println("Initializing SetArmHeightCommand");

        if (m_destination == Constants.ArmDestination.home) {
            m_elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.DISTANCE_BOT);
        } else if (m_destination == Constants.ArmDestination.low) {
            m_elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.DISTANCE_LOW);
        } else if (m_destination == Constants.ArmDestination.middle) {
            m_elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.DISTANCE_MIDDLE);
        } else if (m_destination == Constants.ArmDestination.high) {
            m_elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.DISTANCE_HIGH);
        } else if (m_destination == Constants.ArmDestination.shelf) {
            m_elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.DISTANCE_SHELF);
        } else {
            System.out.println("SetArmHeightCommand - Invalid ArmDestination value");
        }
    }

    @Override
    public void execute() {
        // do nothing
    }

    @Override
    public boolean isFinished() {
        return (m_elevatorSubsystem.atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("SetArmHeightCommand completed");
    }
}
