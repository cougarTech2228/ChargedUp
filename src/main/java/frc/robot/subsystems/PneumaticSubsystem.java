package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PneumaticSubsystem extends SubsystemBase {

    private Solenoid m_gripper;
    private Solenoid m_elevatorBrake;
    private Solenoid m_armBrake;

    PneumaticHub m_pneumaticHub;
    private ShuffleboardTab m_sbTab;

    public PneumaticSubsystem() {
        m_pneumaticHub = new PneumaticHub(Constants.PCM_CAN_ID);

        m_gripper = m_pneumaticHub.makeSolenoid(Constants.GRIPPER_PCM_PORT);
        m_elevatorBrake = m_pneumaticHub.makeSolenoid(Constants.ELEVATOR_BRAKE_PCM_PORT);
        m_armBrake = m_pneumaticHub.makeSolenoid(Constants.ARM_BRAKE_PCM_PORT);

        m_sbTab = Shuffleboard.getTab("Pneumatics (Debug)");

        m_sbTab.addDouble("High Side", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return getHighSidePressure();
            };
        });

        
        m_sbTab.addDouble("Low Side", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return getLowSidePressure();
            };
        });
    }

    public void openGripper() {
        if (!gripperIsOpen()) {
            System.out.println("Open Gripper");
            m_gripper.set(false);
        }
    }

    public void closeGripper() {
        if (gripperIsOpen()) {
            System.out.println("Close Gripper");
            m_gripper.set(true);
        }
    }

    public boolean gripperIsOpen() {
        return !m_gripper.get();
    }

    public void openElevatorBrake() {
        System.out.println("Open Elevator Brake");
        m_elevatorBrake.set(true);
    }

    public void closeElevatorBrake() {
        System.out.println("Close Elevator Brake");
        m_elevatorBrake.set(false);
    }

    public void openArmBrake() {
        System.out.println("Open Arm Brake");
        m_armBrake.set(true);
    }

    public void closeArmBrake() {
        System.out.println("Close Arm Brake");
        m_armBrake.set(false);
    }

    public double getHighSidePressure() {
        return m_pneumaticHub.getPressure(Constants.HIGH_SIDE_ANALOG_PORT);
    }

    public double getLowSidePressure() {
        return m_pneumaticHub.getPressure(Constants.LOW_SIDE_ANALOG_PORT);
    }

    public void toggleGripper() {
        if (gripperIsOpen()) {
            closeGripper();
        } else {
            openGripper();
        }
    }
}
