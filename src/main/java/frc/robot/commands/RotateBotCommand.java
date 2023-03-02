package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RotateBotCommand extends CommandBase {
    private double m_degreesToTurn;
    private double m_angularVelocity;
    private DrivetrainSubsystem m_drivetrainSubsystem;

    double m_currentYawValue;
    double m_startYawValue;

    public RotateBotCommand(double degreesToTurn, double angularVelocity,
            DrivetrainSubsystem drivetrainSubsystem) {

        m_degreesToTurn = degreesToTurn;
        m_angularVelocity = angularVelocity;
        m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {

        System.out.println("RotateBotCommand Command starting");

        if (m_degreesToTurn < 0.0 || m_degreesToTurn > 360.0) {
            System.out.println("ERROR: Degrees value passed into RotateBotCommand out of [0, 360]");
            m_degreesToTurn = 0.0;
        }

        m_currentYawValue = m_drivetrainSubsystem.getYaw();
        m_startYawValue = m_currentYawValue;
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(0.0,
                        0.0,
                        m_angularVelocity * m_drivetrainSubsystem.getRotationalAdjustment()
                                * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        m_drivetrainSubsystem.getGyroscopeRotation()));

        m_currentYawValue = m_drivetrainSubsystem.getYaw();
    }

    @Override
    public boolean isFinished() {
        return true;

        // TODO - don't know if this is right
        // return ((m_currentYawValue <= (m_startYawValue - m_degreesToTurn)) ||
        // (m_currentYawValue >= (m_startYawValue + m_degreesToTurn)));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stopMotors();
        m_drivetrainSubsystem.setMotorsToBrake();

        if (interrupted) {
            System.out.println("RotateBotCommand interrupted");
        } else {
            System.out.println("RotateBotCommand finished normally");
        }
    }
}