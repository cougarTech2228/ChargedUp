package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.DockWithAprilTag;

public class DockWithAprilTagCommand extends CommandBase {
    private XboxController m_xboxController;
    private DrivetrainSubsystem m_drivetrainSubsystem;
    private AprilTagSubsystem m_aprilTagSubsystem;
    private double m_aprilTagId;
    private boolean m_zeroGyro;

    private Runnable m_dockWithAprilTagRunnable;
    private Thread m_dockWithAprilTagThread;

    /** Creates a new ThreadedDockWithAprilTagCommand. */
    public DockWithAprilTagCommand(XboxController xboxController, DrivetrainSubsystem drivetrainSubsystem,
            AprilTagSubsystem aprilTagSubsystem,
            boolean zeroGyro,
            double aprilTagId) {
        m_xboxController = xboxController;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_aprilTagSubsystem = aprilTagSubsystem;
        m_aprilTagId = aprilTagId;
        m_zeroGyro = zeroGyro;

        m_dockWithAprilTagRunnable = new DockWithAprilTag(m_xboxController,
                m_drivetrainSubsystem,
                m_aprilTagSubsystem,
                m_aprilTagId);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Running auto dock with AprilTag command");

        if (m_zeroGyro) {
            m_drivetrainSubsystem.zeroGyroscope();
        }

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
