package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ExtendoSubsystem;

public class SetArmReachCommand extends CommandBase {

    private ExtendoSubsystem m_extendoSubsystem;
    private Constants.ArmDestination m_destination;
    private long m_startTime;
    private long m_timeout = 3;

    public SetArmReachCommand(ExtendoSubsystem extendoSubsystem,
            Constants.ArmDestination destination) {
        m_extendoSubsystem = extendoSubsystem;
        m_destination = destination;

        addRequirements(m_extendoSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Initializing SetArmReachCommand: " + m_destination);

        m_startTime = RobotController.getFPGATime();
        double distance = ExtendoSubsystem.DISTANCE_HOME;

        switch (m_destination) {
            case cone_floor:
                distance = ExtendoSubsystem.DISTANCE_CONE_FLOOR;
                break;
            case cube:
                distance = ExtendoSubsystem.DISTANCE_CUBE;
                break;
            case high:
                distance = ExtendoSubsystem.DISTANCE_HIGH;
                break;
            case home:
                distance = ExtendoSubsystem.DISTANCE_HOME;
                break;
            case low:
                distance = ExtendoSubsystem.DISTANCE_LOW;
                break;
            case middle:
                distance = ExtendoSubsystem.DISTANCE_MIDDLE;
                break;
            case shelf:
                distance = ExtendoSubsystem.DISTANCE_SHELF;
                break;
            case transit:
                distance = ExtendoSubsystem.DISTANCE_TRANSIT;
                break;
        }
        m_extendoSubsystem.goToDistance(distance);
    }

    @Override
    public void execute() {
        // do nothing
    }

    @Override
    public boolean isFinished() {
        long now = RobotController.getFPGATime();
        if (  (now - m_startTime) > m_timeout * 1000000) {
            System.out.println("command timeout");
            return true;
        }
        
        return m_extendoSubsystem.atGoal();
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