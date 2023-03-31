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

        addRequirements(m_elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing SetArmHeightCommand: " + m_destination);
        double height = 0;
        switch (m_destination) {
            case cone_floor:
                height = ElevatorSubsystem.HEIGHT_CONE_FLOOR;
                break;
            case cube:
                height = ElevatorSubsystem.HEIGHT_CUBE;
                break;
            case high:
                height = ElevatorSubsystem.HEIGHT_HIGH;
                break;
            case home:
                height = ElevatorSubsystem.HEIGHT_HOME;
                break;
            case low:
                height = ElevatorSubsystem.HEIGHT_LOW;
                break;
            case middle:
                height = ElevatorSubsystem.HEIGHT_MIDDLE;
                break;
            case shelf:
                height = ElevatorSubsystem.HEIGHT_SHELF;
                break;
            case transit:
                height = ElevatorSubsystem.HEIGHT_TRANSIT;
                break;
            case auto_high:
                height = ElevatorSubsystem.HEIGHT_AUTO_HIGH;
                break;
        }
        m_elevatorSubsystem.setElevatorPosition(height);
    }

    @Override
    public void execute() {
        // do nothing
    }

    @Override
    public boolean isFinished() {
        return (m_elevatorSubsystem.atGoal() ||
                (m_elevatorSubsystem.isElevatorLowerLimitReached() && m_elevatorSubsystem.isStopped()));
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("SetArmHeightCommand interrupted");
        } else {
            System.out.println("SetArmHeightCommand finished normally");
        }
    }
}
