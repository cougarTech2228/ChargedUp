package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.CT_DigitalInput;

public class ArmSubsystem extends SubsystemBase {

    private TalonFX m_winchMotor;
    private TalonFX m_elevatorMotor;

    private Rev2mDistanceSensor m_distMxp;

    private double m_currentArmReachCm = 0.0;
    private double m_currentArmHeightCm = 0.0;

    private CT_DigitalInput m_upperElevatorLimitSwitch;
    private CT_DigitalInput m_lowerElevatorLimitSwitch;
    private CT_DigitalInput m_minimumArmReachLimitSwitch;

    public ArmSubsystem() {
        m_winchMotor = new TalonFX(Constants.ARM_WINCH_MOTOR_ID);
        m_elevatorMotor = new TalonFX(Constants.ARM_ELEVATOR_MOTOR_ID);

        m_distMxp = new Rev2mDistanceSensor(Port.kMXP);

        m_upperElevatorLimitSwitch = new CT_DigitalInput(Constants.UPPER_ELEVATOR_LIMIT_SWITCH_DIO);
        m_lowerElevatorLimitSwitch = new CT_DigitalInput(Constants.LOWER_ELEVATOR_LIMIT_SWITCH_DIO);
        m_minimumArmReachLimitSwitch = new CT_DigitalInput(Constants.MINIMUM_REACH_LIMIT_SWITCH_DIO);
    }

    public void enableDistanceSensor(boolean isEnabled) {
        m_distMxp.setAutomaticMode(isEnabled);
    }

    public void setWinchMotorPercentOutput(double percentOutput) {
        m_winchMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setElevatorMotorPercentOutput(double percentOutput) {
        m_elevatorMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public double getCurrentArmReachCm() {
        return m_currentArmReachCm;
    }

    public double getCurrentArmHeightCm() {
        return m_currentArmHeightCm;
    }

    public void setCurrentArmReachCm(double armReachCm) {
        m_currentArmReachCm = armReachCm;
    }

    public void setCurrentArmHeightCM(double armHeightCM) {
        m_currentArmHeightCm = armHeightCM;
    }

    public boolean isUpperElevatorLimitSwitchActive() {
        return m_upperElevatorLimitSwitch.get();
    }

    public boolean isLowerElevatorLimitSwithActive() {
        return m_lowerElevatorLimitSwitch.get();
    }

    public boolean isMinimumReachLimitSwitchActive() {
        return m_minimumArmReachLimitSwitch.get();
    }

    public void setGripperOpen(boolean open) {
        if (open) {
            // TODO - Call PCH method to open the gripper
            System.out.println("Opening gripper");
        } else {
            // TODO - Call PCH method to close the gripper
            System.out.println("Closing gripper");
        }
    }

    @Override
    public void periodic() {

        // We can use timestamp updates to see if the measurement's value is 'stale'
        if (m_distMxp.isEnabled() && m_distMxp.isRangeValid()) {
            // System.out.println(m_distMxp.getTimestamp() + ": Range: " +
            // m_distMxp.getRange());
            m_currentArmReachCm = m_distMxp.getRange(Unit.kMillimeters) * 10.0;
        }

        //TODO - How will we determine m_currentArmHeightCm (encoder counts per centimeter?)
    }
}
