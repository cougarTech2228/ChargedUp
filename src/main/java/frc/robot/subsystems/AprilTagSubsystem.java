package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utils.CT_LEDStrip.GlowColor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagSubsystem extends SubsystemBase {

    static NetworkTable m_aprilTagTable = NetworkTableInstance.getDefault().getTable("AprilTag");

    static NetworkTableEntry m_pitchEntry = m_aprilTagTable.getEntry("Pitch");
    static NetworkTableEntry m_txEntry = m_aprilTagTable.getEntry("TX");
    static NetworkTableEntry m_tzEntry = m_aprilTagTable.getEntry("TZ");
    static NetworkTableEntry m_tagIdEntry = m_aprilTagTable.getEntry("Tag ID");

    public AprilTagSubsystem() {
    }

    public double getPitch() {
        return m_pitchEntry.getDouble(2228);
    }

    public double getTX() {
        return m_txEntry.getDouble(2228);
    }

    public double getTZ() {
        return m_tzEntry.getDouble(2228);
    }

    public double getTagID() {
        double tagId = m_tagIdEntry.getDouble(2228);

        if (tagId == 1) {
            RobotContainer.getLEDStripSubsystem().glow(GlowColor.Green);
        } else {
            RobotContainer.getLEDStripSubsystem().glow(GlowColor.Red);
        }
        return tagId;
    }

    public long getLastChanged() {
        return m_tagIdEntry.getLastChange();
    }

    @Override
    public void periodic() {
        //System.out.println("TZ: " + getTZ() + " | TagID: " + getTagID());
        //System.out.println("TagID last_change: " + m_tagID.getInfo().last_change);
    }
}