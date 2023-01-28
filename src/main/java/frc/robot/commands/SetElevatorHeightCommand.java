package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetElevatorHeightCommand extends CommandBase {

    private double m_targetHeightCm;
    private double m_currentArmHeightCm;

    private static final double HEIGHT_TOLERANCE_CM = 1.0;
    private static final double ELEVATOR_MOTOR_PERCENT_OUTPUT = 0.10;

    public SetElevatorHeightCommand(double targetHeightCm) {

        // TODO - probably should validate value of targetHeightCm
        m_targetHeightCm = targetHeightCm;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Starting SetElevatorHeightCommand. Target Height(cm): " + m_targetHeightCm);
        m_currentArmHeightCm = RobotContainer.getElevatorSubsystem().getCurrentElevatorHeightCm();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
         // TODO - need to check for correct motor direction
        if (m_currentArmHeightCm > m_targetHeightCm) {
            RobotContainer.getElevatorSubsystem().setElevatorMotorPercentOutput(-ELEVATOR_MOTOR_PERCENT_OUTPUT);
        } else {
            RobotContainer.getElevatorSubsystem().setElevatorMotorPercentOutput(ELEVATOR_MOTOR_PERCENT_OUTPUT);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.getElevatorSubsystem().setElevatorMotorPercentOutput(0.0);
        RobotContainer.getElevatorSubsystem().setMotorToBrake();
        System.out.println("Ending SetElevatorHeightCommand");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // TODO - Can the PID setpoint be set to the proper height?
        // return (RobotContainer.getElevatorSubsystem().atSetpoint());
        return true;
    }
}
