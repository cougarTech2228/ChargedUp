package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PlacePreloadedPieceCommand extends CommandBase {

    Constants.PlacePosition m_placePosition;

    public PlacePreloadedPieceCommand() {
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Starting PlacePreloadedPieceCommand");

        // Based on Shuffleboard selection, create appropriate Place Command for
        // preloaded piece
        Constants.PlacePosition preloadedPieceLevel = RobotContainer.getShuffleboardManager()
                .getPreloadedPieceLevel();
        Constants.ConeOffsetPosition coneOffsetPosition = RobotContainer.getShuffleboardManager().getPreloadedConeOffsetPosition();

        if ((preloadedPieceLevel == Constants.PlacePosition.HighCone) ||
                (preloadedPieceLevel == Constants.PlacePosition.MiddleCone) ||
                (preloadedPieceLevel == Constants.PlacePosition.LowCone)) {
                    if (coneOffsetPosition == Constants.ConeOffsetPosition.Left) {
                        System.out.println("Placing preloaded left cone");
                    } else {
                        System.out.println("Placing preloaded right cone");
                    }
        } else {
           System.out.println("Placing preloaded cube"); 
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending PlacePreloadedPieceCommand ");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
