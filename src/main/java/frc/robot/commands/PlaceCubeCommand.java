package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class PlaceCubeCommand extends CommandBase {

    Constants.PlacePosition m_placePosition;

    public PlaceCubeCommand (Constants.PlacePosition placePosition) {
        m_placePosition = placePosition;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Starting PlaceCubeCommand");

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending PlaceCubeCommand ");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
