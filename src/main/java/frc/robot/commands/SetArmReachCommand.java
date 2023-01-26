package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SetArmReachCommand extends CommandBase {

    private double m_targetReachCm;
    private double m_currentArmReachCm;

    private static final double REACH_TOLERANCE_CM = 1.0;
    private static final double WINCH_MOTOR_PERCENT_OUTPUT = 0.10;
    private static final double MAXIMUM_REACH_CM = Constants.ARM_HIGH_CONE_REACH_CM;

    public SetArmReachCommand(double targetReachCm) {

        m_targetReachCm = targetReachCm;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Starting SetArmHeightCommand. Target Reach(cm): " + m_targetReachCm);

        if (m_targetReachCm > MAXIMUM_REACH_CM) {
            m_targetReachCm = MAXIMUM_REACH_CM;
        }

        RobotContainer.getArmSubsystem().enableDistanceSensor(true);

        m_currentArmReachCm = RobotContainer.getArmSubsystem().getCurrentArmReachCm();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // TODO - need to check for correct motor direction
        // if (m_currentArmReachCm > m_targetReachCm) {
        //     RobotContainer.getArmSubsystem().setWinchMotorPercentOutput(-WINCH_MOTOR_PERCENT_OUTPUT);
        // } else {
        //     RobotContainer.getArmSubsystem().setWinchMotorPercentOutput(WINCH_MOTOR_PERCENT_OUTPUT);
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.getArmSubsystem().enableDistanceSensor(false);
        System.out.println("Ending SetArmReachCommand");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if (RobotContainer.getArmSubsystem().isMinimumReachLimitSwitchActive()) {
        //     return true;
        // } else {
        //     return (Math.abs(m_currentArmReachCm - m_targetReachCm) < REACH_TOLERANCE_CM);
        // }

        return true;
    }
}
