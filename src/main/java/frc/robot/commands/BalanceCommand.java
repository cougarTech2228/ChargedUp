package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
import frc.robot.utils.led.LEDBufferSegment;

public class BalanceCommand extends CommandBase{
    private DrivetrainSubsystem m_drivetrainSubsystem;
    private LEDStripSubsystem m_ledStripSubsystem;

    public BalanceCommand(DrivetrainSubsystem drivetrain){
        m_drivetrainSubsystem = drivetrain;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("BalanceCommand Command starting");
        m_drivetrainSubsystem.setDriveMotorStatusFramePeriod(10);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double roll = m_drivetrainSubsystem.getRoll();
        double speed = 0.1;
        System.out.println("Roll: " + m_drivetrainSubsystem.getRoll() + "; Pitch: " + m_drivetrainSubsystem.getPitch() + "; Yaw: " + m_drivetrainSubsystem.getYaw());

        if(roll > 0){
            speed = -speed;
        }

        if(!(Math.abs(roll) < 2)){
            System.out.println("Drive: " + speed);
            m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speed * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                    0.0,
                    0.0,
                    m_drivetrainSubsystem.getGyroscopeRotation()));
            m_drivetrainSubsystem.getEncoderCount();
        } else{
            m_drivetrainSubsystem.stopMotors();
            m_drivetrainSubsystem.setMotorsToBrake();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stopMotors();
        m_drivetrainSubsystem.setMotorsToBrake();

        // Return the status frame period back to its original value
        m_drivetrainSubsystem.setDriveMotorStatusFramePeriod(20);

        m_ledStripSubsystem.autoPretty();

        System.out.println("BalanceCommand finished");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(m_drivetrainSubsystem.getRoll()) < 1.5);
    }
}
