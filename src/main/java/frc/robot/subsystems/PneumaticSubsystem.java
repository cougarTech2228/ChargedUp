package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PneumaticSubsystem extends SubsystemBase{

    private Solenoid m_gripper;
    PneumaticHub m_pnPneumaticHub;
    private ShuffleboardTab m_sbTab;


    public PneumaticSubsystem(){
        m_pnPneumaticHub = new PneumaticHub(Constants.PCM_CAN_ID);
        m_gripper = m_pnPneumaticHub.makeSolenoid(Constants.GRIPPER_PCM_PORT);
        m_sbTab = Shuffleboard.getTab("Pneumatics");

        m_sbTab.addDouble("Pressure", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return getPressure();
            };
        });
    }

    public void openGripper(){
        System.out.println("Open Gripper");
        m_gripper.set(true);
    }

    public void closeGripper(){
        System.out.println("Close Gripper");
        m_gripper.set(false);
    }

    public boolean gripperIsOpen() {
        return m_gripper.get();
    }

    public double getPressure(){
        return m_pnPneumaticHub.getPressure(0);
    }
}
