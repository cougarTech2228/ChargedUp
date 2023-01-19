package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ChooseOutPathCommand extends CommandBase {

    Constants.PlacePosition m_placePosition;

    public ChooseOutPathCommand() {
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Starting ChooseOutPathCommand");
        HashMap<String, Command> eventMap = new HashMap<>();

        // Based on Shuffleboard selection, create appropriate Place Command for
        // preloaded piece
        Constants.PlacePosition preloadedPieceLevel = RobotContainer.getShuffleboardManager()
                .getPreloadedPieceLevel();
        Constants.ConeOffsetPosition coneOffsetPosition = RobotContainer.getShuffleboardManager()
                .getPreloadedConeOffsetPosition();

        if ((preloadedPieceLevel == Constants.PlacePosition.HighCone) ||
                (preloadedPieceLevel == Constants.PlacePosition.MiddleCone) ||
                (preloadedPieceLevel == Constants.PlacePosition.LowCone)) {
            if (coneOffsetPosition == Constants.ConeOffsetPosition.Left) {
                switch (DriverStation.getLocation()) { //Cone Left
                    case 1:
                        new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto1L_out", eventMap,
                                Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
                        break;
                    case 2:
                        new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto2L_out", eventMap,
                                Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
                        break;
                    case 3:
                        new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto3L_out", eventMap,
                                Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
                        break;
                    default:
                        System.out.println("Uh Oh, somthing went wrong :( (Auto Location not found)");
                        System.out.println("Invalid position received from DriverStation");
                }
            } else { //Cone Right
                switch (DriverStation.getLocation()) {
                    case 1:
                        new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto1R_out", eventMap,
                                Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
                        break;
                    case 2:
                        new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto2R_out", eventMap,
                                Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
                        break;
                    case 3:
                        new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto3R_out", eventMap,
                                Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
                        break;
                    default:
                        System.out.println("Uh Oh, somthing went wrong :( (Auto Location not found)");
                        System.out.println("Invalid position received from DriverStation");
                }
            }
        } else { //Cube
            switch (DriverStation.getLocation()) {
                case 1:
                    new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "autoM_out", eventMap,
                            Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
                    break;
                case 2:
                    new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto2M_out", eventMap,
                            Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
                    break;
                case 3:
                    new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "auto3M_out", eventMap,
                            Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true);
                    break;
                default:
                    System.out.println("Uh Oh, somthing went wrong :( (Auto Location not found)");
                    System.out.println("Invalid position received from DriverStation");
            }
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending ChooseOutPathCommand ");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
