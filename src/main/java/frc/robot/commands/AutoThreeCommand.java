package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmDestination;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class AutoThreeCommand extends SequentialCommandGroup {

    private double m_startTime = 0;
    private static ElevatorSubsystem m_elevatorSubsystem;
    private static ExtendoSubsystem m_extendoSubsystem;
    private static DrivetrainSubsystem m_drivetrainSubsystem;
    private static PneumaticSubsystem m_pneumaticSubsystem;

    public AutoThreeCommand(ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem,
            DrivetrainSubsystem drivetrainSubystem,
            PneumaticSubsystem pneumaticSubsystem) {

        m_elevatorSubsystem = elevatorSubsystem;
        m_extendoSubsystem = extendoSubsystem;
        m_drivetrainSubsystem = drivetrainSubystem;
        m_pneumaticSubsystem = pneumaticSubsystem;

        HashMap<String, Command> m_eventMap = new HashMap<>();

        addCommands(
            new InstantCommand(() -> printStartCommand()),
            new InstantCommand(m_drivetrainSubsystem::zeroGyroscope),
            new InstantCommand(m_drivetrainSubsystem::setMotorsToBrake),
            new SequentialCommandGroup(
                new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.auto_high),
                new SetArmReachCommand(m_extendoSubsystem, ArmDestination.high),
                new WaitCommand(.25),
                new InstantCommand(() -> m_pneumaticSubsystem.openGripper())
            ),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home),
                    new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                    new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home),
                    new InstantCommand(() -> m_pneumaticSubsystem.openGripper())
                ),
                new FollowTrajectoryCommand(m_drivetrainSubsystem, "auto3_NoReturn", m_eventMap,
                        4, 3, true)
            ),
            new InstantCommand(() -> m_drivetrainSubsystem.reverseGyroscope()),
            new InstantCommand(() -> printEndCommand()));
    }

    private void printStartCommand() {
        m_startTime = Timer.getFPGATimestamp();
        System.out.println("Starting AutoThreeCommand");
    }

    private void printEndCommand() {
        System.out.println("AutoThreeCommand completed in " + (Timer.getFPGATimestamp() - m_startTime) + " seconds");
    }
}
