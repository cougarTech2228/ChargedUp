package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    TalonFX m_winchMotor = new TalonFX(Constants.ARM_WINCH_MOTOR_ID);

    public ArmSubsystem() {
    }

    public void setWinchMotorPercentOutput(double percentOutput) {
        m_winchMotor.set(ControlMode.PercentOutput, percentOutput);
    }
    
    @Override
    public void periodic() {
        //System.out.println("TZ: " + getTZ() + " | TagID: " + getTagID());
        //System.out.println("TagID last_change: " + m_tagID.getInfo().last_change);
    }
}
