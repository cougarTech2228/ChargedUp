package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;

public class SerialArmCommand extends CommandBase {
    private ExtendoSubsystem m_extendoSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;
    private Constants.ArmDestination m_destination;
    private double safteyValue = 25;

    public SerialArmCommand(ExtendoSubsystem extendoSubsystem, ElevatorSubsystem elevatorSubsystem,
            Constants.ArmDestination destination) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_extendoSubsystem = extendoSubsystem;
        m_destination = destination;
    }

    @Override
    public void initialize() {
        System.out.println("Initializing ArmCommand");
        if (m_destination == Constants.ArmDestination.bot) {
            m_extendoSubsystem.goToDistanceCM(ExtendoSubsystem.DISTANCE_BOT);
        }
        flag = false;
    }

    @Override
    public boolean isFinished() {
        return (m_elevatorSubsystem.atGoal() && m_extendoSubsystem.atGoal());
    }

    private boolean flag = false;

    @Override
    public void execute() {
        if (m_destination == Constants.ArmDestination.bot) {
            if (m_extendoSubsystem.getCurrentArmReachCm() <= safteyValue) {
                if (!flag) {
                    m_elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.DISTANCE_BOT);
                    flag = true;
                }
            }
        }
    }
}
