package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.DockWithAprilTag;

public class DockWithAprilTagCommand extends CommandBase {
    private double m_aprilTagId;
    private boolean m_isCameraForward;

    private Runnable m_dockWithAprilTagRunnable;
    private Thread m_dockWithAprilTagThread;

    /** Creates a new ThreadedDockWithAprilTagCommand. */
    public DockWithAprilTagCommand(
            boolean isCameraForward) {
        m_isCameraForward = isCameraForward;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        switch (DriverStation.getLocation()) {
            case 1:
                if (DriverStation.getAlliance() == Alliance.Blue) {
                    m_aprilTagId = 6.0;
                } else {
                    m_aprilTagId = 1.0;
                }
                break;
            case 2:
                if (DriverStation.getAlliance() == Alliance.Blue) {
                    m_aprilTagId = 7.0;
                } else {
                    m_aprilTagId = 2.0;
                }
                break;
            case 3:
                if (DriverStation.getAlliance() == Alliance.Blue) {
                    m_aprilTagId = 3.0; //TODO Fix this when red path is made 8.0;
                } else {
                    m_aprilTagId = 3.0;
                }
                break;
            default:
                System.out.println("Invalid position received from DriverStation");
        }

        System.out.println("Running auto dock with AprilTag command for tag ID: " + m_aprilTagId);

        m_dockWithAprilTagRunnable = new DockWithAprilTag(
                m_isCameraForward,
                m_aprilTagId);

        m_dockWithAprilTagThread = new Thread(m_dockWithAprilTagRunnable, "DockWithAprilTagThread");
        m_dockWithAprilTagThread.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // do nothing ... code is running in a thread
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("ending auto dock with AprilTag command");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (!m_dockWithAprilTagThread.isAlive());
    }
}
