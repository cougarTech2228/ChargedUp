package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmDestination;
import frc.robot.commands.DriveFwdRevCommand;
import frc.robot.commands.SetArmHeightCommand;
import frc.robot.commands.SetArmReachCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class PlacePreloadedPieceCommandChooser {

    Constants.PlacePosition m_pieceLevel;
    private static ElevatorSubsystem m_elevatorSubsystem;
    private static ExtendoSubsystem m_extendoSubsystem;
    private static PneumaticSubsystem m_pneumaticSubsystem;
    private static DrivetrainSubsystem m_drivetrainSubsystem;

    public PlacePreloadedPieceCommandChooser(ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem,
            PneumaticSubsystem pneumaticSubsystem, DrivetrainSubsystem drivetrainSubsystem,
            Constants.PlacePosition pieceLevel) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_extendoSubsystem = extendoSubsystem;
        m_pneumaticSubsystem = pneumaticSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_pieceLevel = pieceLevel;
    }

    public SequentialCommandGroup getPlacePieceCommand() {

        if (m_pieceLevel == Constants.PlacePosition.HighCone || m_pieceLevel == Constants.PlacePosition.HighCube) {
            return new SequentialCommandGroup(
                    new SetArmHeightCommand(m_elevatorSubsystem,
                            ArmDestination.high),
                    new SetArmReachCommand(m_extendoSubsystem,
                            ArmDestination.high),
                    new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                    new SetArmReachCommand(m_extendoSubsystem,
                            ArmDestination.home),
                    new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                    new SetArmHeightCommand(m_elevatorSubsystem,
                            ArmDestination.home));

        } else if (m_pieceLevel == Constants.PlacePosition.MiddleCone
                || m_pieceLevel == Constants.PlacePosition.MiddleCube) {
            return new SequentialCommandGroup(
                    new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.middle),
                    new SetArmReachCommand(m_extendoSubsystem, ArmDestination.middle),
                    new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                    new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home),
                    new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                    new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home));

        } else if (m_pieceLevel == Constants.PlacePosition.LowCone) {
            return new SequentialCommandGroup(
                    new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.preloaded_cone),
                    new SetArmReachCommand(m_extendoSubsystem, ArmDestination.low),
                    new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.low),
                    new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                    new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home),
                    new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                    new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home));

        } else if (m_pieceLevel == Constants.PlacePosition.LowCube) {
            return new SequentialCommandGroup(
                    new SetArmReachCommand(m_extendoSubsystem, ArmDestination.tight),
                    new DriveFwdRevCommand(Constants.GRID_BACK_OFF_DISTANCE_CM,
                            Constants.GRID_BACK_OFF_SPEED, m_drivetrainSubsystem),
                    new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.low),
                    new DriveFwdRevCommand(Constants.GRID_BACK_OFF_DISTANCE_CM,
                            -Constants.GRID_BACK_OFF_SPEED, m_drivetrainSubsystem),
                    new SetArmReachCommand(m_extendoSubsystem, ArmDestination.low),
                    new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                    new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home),
                    new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                    new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home));

        } else {
            System.out.println("Error selecting place position");
        }

        return null;
    }
}
