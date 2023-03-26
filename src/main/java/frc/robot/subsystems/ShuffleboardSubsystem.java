package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShuffleboardSubsystem extends SubsystemBase {

    private static ShuffleboardTab m_chargedUpTab;
    private GenericEntry m_speedSlider;

    private static SendableChooser<Constants.AutoPosition> m_positionChooser = new SendableChooser<>();

    public ShuffleboardSubsystem(ShuffleboardTab chargedUpTab) {

        m_chargedUpTab = chargedUpTab;
    }

    public void configureShuffleboard() {

        m_positionChooser.setDefaultOption("Position 1", Constants.AutoPosition.Position1);
        m_positionChooser.addOption("Position 2", Constants.AutoPosition.Position2);
        m_positionChooser.addOption("Position 3", Constants.AutoPosition.Position3);

        m_chargedUpTab.add("Auto: Field Position", m_positionChooser)
                .withWidget(BuiltInWidgets.kSplitButtonChooser)
                .withSize(3, 1)
                .withPosition(0, 0);

        m_chargedUpTab.addBoolean("Gripper Open?", new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return RobotContainer.getPneumaticSubsystem().gripperIsOpen();
            };
        }).withSize(1, 1).withPosition(0, 1);

        m_chargedUpTab.addDouble("Arm Height (cm):", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return RobotContainer.getElevatorSubsystem().getMeasurement();
            };
        }).withSize(1, 1).withPosition(1, 1);

        m_chargedUpTab.addDouble("Arm Reach (cm):", new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return RobotContainer.getExtendoSubsystem().getCurrentArmReach();
            };
        }).withSize(1, 1).withPosition(2, 1);

        m_speedSlider = m_chargedUpTab.add("Max Speed", 1)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
        

        // HttpCamera httpCamera = new HttpCamera("DriverVision", "http://wpilibpi.local:1181/stream.mjpg");
        // CameraServer.addCamera(httpCamera);
        // m_chargedUpTab.add(httpCamera).withWidget(BuiltInWidgets.kCameraStream)
        //         .withSize(3, 3)
        //         .withPosition(3, 0);
    }

    public Constants.AutoPosition getAutoPosition() {
        return m_positionChooser.getSelected();
    }

    public double getSpeedSlider(){
        return m_speedSlider.get().getDouble();
    }

    @Override
    public void periodic() {
    }
}