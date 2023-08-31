package frc.robot.commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.AcquirerSubsystem;

public class AcquirerCommand extends CommandBase{
    private AcquirerSubsystem m_acquirerSubsystem;
    private boolean m_isSpitting; //true = spit, false = suck
    private double currentCutOff = 1000; //random guess just to give it a value
    private double spitTimeCutOff = 3000; //random guess just to give it a value
    private double spitStartTime;

    public AcquirerCommand(AcquirerSubsystem acquirerSubsystem, boolean spitting){
        m_acquirerSubsystem = acquirerSubsystem;
        m_isSpitting = spitting;
    }

    @Override
    public void initialize() {
        System.out.println("Initializeing Acquirer Command");

        if(m_isSpitting){
            System.out.println("Spitting Object");
            spitStartTime = Timer.getFPGATimestamp();
            m_acquirerSubsystem.spitPiece();
        }
        else{
            System.out.println("Sucking Object");
            m_acquirerSubsystem.suckPiece();
        }
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        if(m_acquirerSubsystem.getMotorCurrent() > currentCutOff){
            return true;
        }
        else if(m_isSpitting && (Timer.getFPGATimestamp() - spitStartTime >= spitTimeCutOff)){
            return true;
        }
        else{
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_acquirerSubsystem.stopMotors();
    }

}
