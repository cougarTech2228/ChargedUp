package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PlaceStagedPieceCommand extends CommandBase {

    Constants.PlacePosition m_placePosition;

    public PlaceStagedPieceCommand() {
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Starting PlaceStagedPieceCommand");

        // Based on Shuffleboard selection, create appropriate Place Command for
        // preloaded piece
        Constants.PlacePosition stagedPieceLevel = RobotContainer.getShuffleboardManager()
                .getPreloadedPieceLevel();
        Constants.ConeOffsetPosition coneOffsetPosition = RobotContainer.getShuffleboardManager().getStagedConeOffsetPosition();


        if ((stagedPieceLevel == Constants.PlacePosition.HighCone) ||
                (stagedPieceLevel == Constants.PlacePosition.MiddleCone) ||
                (stagedPieceLevel == Constants.PlacePosition.LowCone)) {
                    if (coneOffsetPosition == Constants.ConeOffsetPosition.Left) {
                        System.out.println("Placing staged left cone ... need to strafe left");
                    } else {
                        System.out.println("Placing staged right cone ... need to strafe right");
                    }
        } else {
           System.out.println("Placing staged cube"); 
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending PlaceStagedPieceCommand ");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
