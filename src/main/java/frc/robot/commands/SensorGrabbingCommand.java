package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmDestination;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class SensorGrabbingCommand extends CommandBase{
    PneumaticSubsystem m_pneumaticSubsystem;
    ExtendoSubsystem m_extendoSubsystem;
    ElevatorSubsystem m_elevatorSubsystem;

    DigitalInput m_sensor;

    public SensorGrabbingCommand(){
        m_sensor = new DigitalInput(Constants.GRABBING_SENSOR_DIO);
    }

    @Override
    public void initialize() {
        new SequentialCommandGroup(
            new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
            new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home),
            new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home),
            new InstantCommand(() -> m_pneumaticSubsystem.openGripper())
        );
    }

    @Override
    public void execute() {
        // do nothing
    }

    @Override
    public boolean isFinished() {
        if(m_sensor.get()){
            return true;
        } else {
            return false;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        new InstantCommand(() -> m_pneumaticSubsystem.closeGripper());
    }
}
