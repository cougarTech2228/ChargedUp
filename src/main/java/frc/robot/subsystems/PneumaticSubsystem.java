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

    private enum BrakeState {
        unknown,
        closed,
        open
    }
    private BrakeState m_armBrakeState = BrakeState.unknown;

    PneumaticHub m_pneumaticHub;
    LEDStripSubsystem m_ledStripSubsystem;
    private ShuffleboardTab m_sbTab;

    public PneumaticSubsystem(LEDStripSubsystem ledStripSubsystem) {
        m_pneumaticHub = new PneumaticHub(Constants.PCM_CAN_ID);

        m_ledStripSubsystem = ledStripSubsystem;

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
        System.out.println("Open Gripper");
        m_gripper.set(true);
        m_ledStripSubsystem.gripperLights(true);
    }

    public void closeGripper() {
        System.out.println("Close Gripper");
        m_gripper.set(false);
        m_ledStripSubsystem.gripperLights(false);
    }

    public boolean gripperIsOpen() {
        return !m_gripper.get();
    }

    public void openElevatorBrake() {
        // System.out.println("Open Elevator Brake");
        m_elevatorBrake.set(true);
    }

    public void closeElevatorBrake() {
        // System.out.println("Close Elevator Brake");
        m_elevatorBrake.set(false);
    }

    public void openArmBrake() {
        System.out.println("Open Arm Brake: " + m_armBrakeState);
        if (m_armBrakeState != BrakeState.open) {
            m_armBrakeState = BrakeState.open;
            m_armBrake.set(true);
        }
    }

    public void closeArmBrake() {
       System.out.println("Close Arm Brake: " + m_armBrakeState);
        //m_armBrake.set(true);
        if (m_armBrakeState != BrakeState.closed) {
            m_armBrakeState = BrakeState.closed;
            m_armBrake.set(false);
        }
    }

    public void toggleArmBrake() {
        m_armBrake.toggle();
    }

    public double getHighSidePressure() {
        return m_pneumaticHub.getPressure(Constants.HIGH_SIDE_ANALOG_PORT);
    }

    public double getLowSidePressure() {
        return m_pneumaticHub.getPressure(Constants.LOW_SIDE_ANALOG_PORT);
    }

    public void toggleGripper() {
        System.out.println("gripper toggle");
        m_gripper.toggle();
    }
}
