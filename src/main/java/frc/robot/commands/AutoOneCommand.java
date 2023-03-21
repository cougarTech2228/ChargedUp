package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmDestination;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class AutoOneCommand extends SequentialCommandGroup {

    private static ElevatorSubsystem m_elevatorSubsystem;
    private static ExtendoSubsystem m_extendoSubsystem;
    private static DrivetrainSubsystem m_drivetrainSubsystem;
    private static PneumaticSubsystem m_pneumaticSubsystem;

    private double m_startTime = 0;

    public AutoOneCommand(ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem,
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
                new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.high),
                new SetArmReachCommand(m_extendoSubsystem, ArmDestination.high),
                new WaitCommand(0.25)
            ),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                    new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home),
                    new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                    new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home),
                    new InstantCommand(() -> m_pneumaticSubsystem.openGripper())
                ),
                new FollowTrajectoryCommand(m_drivetrainSubsystem, "auto1_NoReturn", m_eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true)
            ),
            new InstantCommand(() -> m_drivetrainSubsystem.reverseGyroscope()),
            new InstantCommand(() -> printEndCommand())
        );
    }

    private void printStartCommand() {
        m_startTime = Timer.getFPGATimestamp();
        System.out.println("Starting AutoOneCommand");
    }

    private void printEndCommand() {
        System.out.println("AutoOneCommand completed in " + (Timer.getFPGATimestamp() - m_startTime) + " seconds");
    }
}
