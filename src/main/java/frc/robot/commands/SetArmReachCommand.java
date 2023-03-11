package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ExtendoSubsystem;

public class SetArmReachCommand extends CommandBase {

    private ExtendoSubsystem m_extendoSubsystem;
    private Constants.ArmDestination m_destination;

    public SetArmReachCommand(ExtendoSubsystem extendoSubsystem,
            Constants.ArmDestination destination) {
        m_extendoSubsystem = extendoSubsystem;
        m_destination = destination;

        addRequirements(m_extendoSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing SetArmReachCommand");

        if (m_destination == Constants.ArmDestination.home) {
            m_extendoSubsystem.goToDistanceCM(ExtendoSubsystem.DISTANCE_HOME);
        } else if (m_destination == Constants.ArmDestination.low) {
            m_extendoSubsystem.goToDistanceCM(ExtendoSubsystem.DISTANCE_LOW);
        } else if (m_destination == Constants.ArmDestination.middle) {
            m_extendoSubsystem.goToDistanceCM(ExtendoSubsystem.DISTANCE_MIDDLE);
        } else if (m_destination == Constants.ArmDestination.high) {
            m_extendoSubsystem.goToDistanceCM(ExtendoSubsystem.DISTANCE_HIGH);
        } else if (m_destination == Constants.ArmDestination.shelf) {
            m_extendoSubsystem.goToDistanceCM(ExtendoSubsystem.DISTANCE_SHELF);
        } else if (m_destination == Constants.ArmDestination.transit) {
            m_extendoSubsystem.goToDistanceCM(ExtendoSubsystem.DISTANCE_TRANSIT);
        }else {
            System.out.println("SetArmHeightCommand - Invalid ArmDestination value");
        }
    }

    @Override
    public void execute() {
        // do nothing
    }

    @Override
    public boolean isFinished() {
        return (m_extendoSubsystem.atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("SetArmReachCommand interrupted");
        } else {
            System.out.println("SetArmReachCommand finished normally");
        }
    }

}