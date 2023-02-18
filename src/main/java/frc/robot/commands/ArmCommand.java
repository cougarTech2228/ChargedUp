package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;

public class ArmCommand extends CommandBase {
    private ExtendoSubsystem m_extendoSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;
    private Destination m_destination;
    private double safteyValue = 25;

    public enum Destination {
        high,
        mid,
        low,
        bot
    }

    public ArmCommand(ExtendoSubsystem extendoSubsystem, ElevatorSubsystem elevatorSubsystem,
            Destination destination) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_extendoSubsystem = extendoSubsystem;
        m_destination = destination;
    }

    @Override
    public void initialize() {
        // super.initialize();
        if (m_destination == Destination.bot) {
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
        // super.execute();
        if (m_destination == Destination.bot) {
            if (m_extendoSubsystem.getCurrentArmReachCm() <= safteyValue) {
                System.out.println("Inside execute if");
                if (!flag) {
                    m_elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.DISTANCE_BOT);
                    flag = true;
                }
            }
        }
    }
}
