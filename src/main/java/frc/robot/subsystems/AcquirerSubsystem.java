package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AcquirerSubsystem extends SubsystemBase {

  private CANSparkMax m_leadMotor;
  private CANSparkMax m_followMotor;
  private double speed = .8; //random guess just to give it a value

  public AcquirerSubsystem() {
    m_leadMotor = new CANSparkMax(Constants.ACQUIRER_RIGHT_ID, MotorType.kBrushless);
    m_followMotor = new CANSparkMax(Constants.ACQUIRER_LEFT_ID, MotorType.kBrushless);

    m_followMotor.follow(m_leadMotor, true); //invert
  }

  public void suckPiece(){
    m_leadMotor.set(speed);
  }

  public void spitPiece(){
    m_leadMotor.set(-speed);
  }

  public void stopMotors(){
    m_leadMotor.stopMotor();
    m_followMotor.stopMotor();
  }

  public double getMotorCurrent(){
    return m_leadMotor.getOutputCurrent();
  }

  @Override
  public void periodic(){
   
  }
}
