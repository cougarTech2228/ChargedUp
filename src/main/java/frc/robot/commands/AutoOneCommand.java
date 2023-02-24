package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.utils.AprilTagManager;
import frc.robot.utils.OutPathFileNameChooser;
import frc.robot.utils.PlacePreloadedPieceCommandChooser;
import frc.robot.utils.PlaceStagedPieceCommandChooser;

public class AutoOneCommand extends SequentialCommandGroup {

    private static ElevatorSubsystem m_elevatorSubsystem;
    private static ExtendoSubsystem m_extendoSubsystem;
    private static DrivetrainSubsystem m_drivetrainSubsystem;
    private static ShuffleboardSubsystem m_shuffleboardSubsystem;
    private static AprilTagManager m_aprilTagManager;
    private static PneumaticSubsystem m_pneumaticSubsystem;

    private enum CommandSelector {

        STRAFE_LEFT,
        STRAFE_RIGHT,
        STRAFE_NONE
    }

    private double m_startTime = 0;

    public AutoOneCommand(ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem,
            DrivetrainSubsystem drivetrainSubystem, ShuffleboardSubsystem shuffleboardSubsystem,
            AprilTagManager aprilTagManager, PneumaticSubsystem pneumaticSubsystem) {

        m_elevatorSubsystem = elevatorSubsystem;
        m_extendoSubsystem = extendoSubsystem;
        m_drivetrainSubsystem = drivetrainSubystem;
        m_shuffleboardSubsystem = shuffleboardSubsystem;
        m_aprilTagManager = aprilTagManager;
        m_pneumaticSubsystem = pneumaticSubsystem;

        // TODO - do we want to do something cool at each stage like with LEDs?
        // We could create multiple eventMaps
        HashMap<String, Command> m_eventMap = new HashMap<>();

        OutPathFileNameChooser m_outPathFileNameChooser = new OutPathFileNameChooser(m_shuffleboardSubsystem);
        String m_outPathFileName = m_outPathFileNameChooser.getOutPathFileName();

        // Get the appropriate command group to place the Preloaded Game Piece
        PlacePreloadedPieceCommandChooser m_placePreloadedPieceCommandChooser = new PlacePreloadedPieceCommandChooser(
                m_elevatorSubsystem, m_extendoSubsystem, m_pneumaticSubsystem, m_shuffleboardSubsystem
                        .getPreloadedPieceLevel());
        SequentialCommandGroup m_placePreloadedPieceSequentialCommandGroup = m_placePreloadedPieceCommandChooser
                .getPlacePieceCommand();

        // Get the appropriate command group to place the Staged Game Piece
        PlaceStagedPieceCommandChooser m_placeStagedPieceCommandChooser = new PlaceStagedPieceCommandChooser(
                m_elevatorSubsystem, m_extendoSubsystem, m_pneumaticSubsystem, m_shuffleboardSubsystem
                        .getStagedPieceLevel());
        SequentialCommandGroup m_placeStagedPieceSequentialCommandGroup = m_placeStagedPieceCommandChooser
                .getPlacePieceCommand();

        addCommands(new InstantCommand(() -> printStartCommand()),
                new InstantCommand(m_drivetrainSubsystem::zeroGyroscope),
                new InstantCommand(m_drivetrainSubsystem::setMotorsToBrake),
                m_placePreloadedPieceSequentialCommandGroup,
                /*new FollowTrajectoryCommand(m_drivetrainSubsystem, m_outPathFileName,
                        m_eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new FollowTrajectoryCommand(m_drivetrainSubsystem, "auto1_back",
                        m_eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new DockWithAprilTagCommand(false, m_shuffleboardSubsystem, m_aprilTagManager, m_drivetrainSubsystem),
                new WaitCommand(Constants.WAIT_TIME_AFTER_APRIL_TAG_DOCK_S), // Let the Network Table updates settle a
                                                                             // bit
                new SelectCommand(
                        Map.ofEntries(
                                Map.entry(CommandSelector.STRAFE_LEFT,
                                        new StrafeCommand(Constants.GRID_STRAFE_DISTANCE, -Constants.STRAFE_SPEED,
                                                true, m_drivetrainSubsystem, m_aprilTagManager)),
                                Map.entry(CommandSelector.STRAFE_RIGHT,
                                        new StrafeCommand(Constants.GRID_STRAFE_DISTANCE, Constants.STRAFE_SPEED,
                                                true, m_drivetrainSubsystem, m_aprilTagManager)),
                                Map.entry(CommandSelector.STRAFE_NONE,
                                        new PrintCommand("We're already lined up, no strafing necessary"))),
                        this::selectStagedStrafe),
                m_placeStagedPieceSequentialCommandGroup,*/
                new InstantCommand(() -> m_drivetrainSubsystem.reverseGyroscope()),
                new InstantCommand(() -> printEndCommand()));
    }

    // Choose whether or not we have to strafe and in what direction based on
    // Shuffleboard inputs
    private CommandSelector selectStagedStrafe() {

        Constants.PlacePosition placePosition = m_shuffleboardSubsystem.getStagedPieceLevel();
        Constants.ConeOffsetPosition conePosition = m_shuffleboardSubsystem
                .getStagedConeOffsetPosition();

        if ((placePosition == Constants.PlacePosition.HighCone) ||
                (placePosition == Constants.PlacePosition.MiddleCone) ||
                (placePosition == Constants.PlacePosition.LowCone)) {
            if (conePosition == Constants.ConeOffsetPosition.Left) {
                return CommandSelector.STRAFE_LEFT;
            } else {
                return CommandSelector.STRAFE_RIGHT;
            }
        } else {
            return CommandSelector.STRAFE_NONE;
        }
    }

    private void printStartCommand() {
        m_startTime = Timer.getFPGATimestamp();
        System.out.println("Starting AutoOneCommand");
    }

    private void printEndCommand() {
        System.out.println("AutoOneCommand completed in " + (Timer.getFPGATimestamp() - m_startTime) + " seconds");
    }
}
