package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StrafeCommand extends CommandBase {

    private double m_targetStrafeDistance = 0.0;
    private double m_currentStrafeDistance = 0.0;

    public StrafeCommand(double targetStrafeDistance) {
        m_targetStrafeDistance = targetStrafeDistance;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Starting StrafeCommand");

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("Moving " + m_targetStrafeDistance + " cm");

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending StrafeCommand ");
    }

    // Returns true when the command should end.
    // @Override
    public boolean isFinished() {
        // TODO need to work on how we're going to say we're done, comparing two doubles isn't good.
        //return (m_currentStrafeDistance == m_targetStrafeDistance);
        return true;
    }
}
