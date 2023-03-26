package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmDestination;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class AutoTwoCommand extends SequentialCommandGroup {

    private double m_startTime = 0;
    private static ElevatorSubsystem m_elevatorSubsystem;
    private static ExtendoSubsystem m_extendoSubsystem;
    private static DrivetrainSubsystem m_drivetrainSubsystem;
    private static PneumaticSubsystem m_pneumaticSubsystem;

    public AutoTwoCommand(ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem,
            DrivetrainSubsystem drivetrainSubystem,
            PneumaticSubsystem pneumaticSubsystem,
            LEDStripSubsystem ledStripSubsystem) {

        m_elevatorSubsystem = elevatorSubsystem;
        m_extendoSubsystem = extendoSubsystem;
        m_drivetrainSubsystem = drivetrainSubystem;
        m_pneumaticSubsystem = pneumaticSubsystem;

        // HashMap<String, Command> m_eventMap = new HashMap<>();

        // addCommands(
        //     new InstantCommand(() -> printStartCommand()),
        //     new InstantCommand(m_drivetrainSubsystem::zeroGyroscope),
        //     new InstantCommand(m_drivetrainSubsystem::setMotorsToBrake),
        //     new SequentialCommandGroup(
        //         new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.high),
        //         new SetArmReachCommand(m_extendoSubsystem, ArmDestination.high),
        //         new InstantCommand(() -> m_pneumaticSubsystem.openGripper())),
        //     new ParallelCommandGroup(
        //         new SequentialCommandGroup(
        //             new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home),
        //             new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
        //             new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home)),
        //         new FollowTrajectoryCommand(m_drivetrainSubsystem, "auto2", m_eventMap,
        //                 4, 3, true)),
        //     new BalanceCommand(m_drivetrainSubsystem),
        //     new RotateBotCommand(1, 1, m_drivetrainSubsystem),
        //     new InstantCommand(() -> m_drivetrainSubsystem.reverseGyroscope()),
        //     new InstantCommand(() -> printEndCommand()));

        addCommands(
            new InstantCommand(() -> printStartCommand()),
            new InstantCommand(m_drivetrainSubsystem::zeroGyroscope),
            new InstantCommand(m_drivetrainSubsystem::setMotorsToBrake),
            new SequentialCommandGroup(
                new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                new WaitCommand(.25),
                new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.high),
                new SetArmReachCommand(m_extendoSubsystem, ArmDestination.high),
                new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                new WaitCommand(.25)
            ),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home),
                    new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                    new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home)
                ),
                new BalanceCommandEnhanced(drivetrainSubystem, ledStripSubsystem)
            ),
            new InstantCommand(() -> m_drivetrainSubsystem.reverseGyroscope()),
            new InstantCommand(() -> printEndCommand())
        );
    }

    private void printStartCommand() {
        m_startTime = Timer.getFPGATimestamp();
        System.out.println("Starting AutoTwoCommand");
    }

    private void printEndCommand() {
        System.out.println("AutoTwoCommand completed in " + (Timer.getFPGATimestamp() - m_startTime) + " seconds");
    }
}
