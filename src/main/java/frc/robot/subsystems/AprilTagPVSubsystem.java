package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class AprilTagPVSubsystem extends SubsystemBase {

    // The cameraName must match the Camera string in the PhotonVision Dashboard
    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    // Need to change this for the competition bot obviously.
    static double CAMERA_HEIGHT_METERS = 0.23;

    // Since I'm not a human protractor and couldn't determine the precise pitch of
    // the camera, I placed the robot at a known distance away from the Cube Shelf
    // target and kept tweaking the pitch value below until the Range debug gave me
    // the correct distance.
    static double CAMERA_PITCH_RADIANS = 0.24;

    // Cube Shelf target height = 15.3" + 3" to center of target
    static double CUBE_SHELF_TARGET_HEIGHT_METERS = Units.inchesToMeters(18.13);

    // Substation target heigh = 24.38" + 3" to center of target
    static double SUBSTATION_TARGET_HEIGHT_METERS = Units.inchesToMeters(24.38);

    double m_targetHeightMeters = CUBE_SHELF_TARGET_HEIGHT_METERS;
    double m_range = 0.0;
    double m_sidewaysOffset = 0.0;
    int m_tagId = 2228;
    int m_missedTagDetections = 0;

    double m_periodicDebugCounter = 0.0;

    public AprilTagPVSubsystem() {
    }

    public void switchToCubeShelfTarget() {
        m_targetHeightMeters = CUBE_SHELF_TARGET_HEIGHT_METERS;
    }

    public void switchToSubstationTarget() {
        m_targetHeightMeters = SUBSTATION_TARGET_HEIGHT_METERS;
    }

    public double getRange() {
        return m_range;
    }

    public double getSidewaysOffset() {
        return m_sidewaysOffset;
    }

    public double getTagID() {
        return m_tagId;
    }

    @Override
    public void periodic() {
        // var result  = camera.getLatestResult();

        // if (result.hasTargets()) {

        //     m_missedTagDetections = 0;

        //     PhotonTrackedTarget bestTarget = result.getBestTarget();

        //     m_tagId = bestTarget.getFiducialId();

        //     // This is the value for "X" shown on the PV Dashboard
        //     double calc_range = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS,
        //             m_targetHeightMeters,
        //             CAMERA_PITCH_RADIANS,
        //             Units.degreesToRadians(bestTarget.getPitch()));

        //     Transform3d transform3d = bestTarget.getBestCameraToTarget();

        //     // These should get the values from the Translation Vector
        //     m_range = transform3d.getX();
        //     m_sidewaysOffset = -transform3d.getY();

        //     // Only dump the debug once per second
        //     if (m_periodicDebugCounter++ > 50) {
        //         m_periodicDebugCounter = 0;

        //         //System.out.printf("ID: %d Range: %.2f Yaw: %.2f\n", m_tagId, m_range, m_yaw);
        //         System.out.printf("ID: %d Range: %.2f CalcRange: %.2f Offset: %.2f\n", m_tagId, calc_range, m_range, m_sidewaysOffset);
        //     }           
        // } else {
        //     if (m_missedTagDetections++ > 25) {
        //         // m_missedTagDetections gets reset when a good tag is found
        //         m_tagId = 2228;
        //     }
        // }
    }
}