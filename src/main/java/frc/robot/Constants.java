// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.525; // Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.525; // Measure and set wheelbase

    // CAN ID Assignments
    // Since CAN messages are prioritized based on CAN ID, we are setting the CAN
    // Coder IDs
    // to lower values in an attempt to get rid of the "dead wheel" issue we
    // occasionally are
    // seeing at startup.
    public static final double LEFT_ALIGN_ADJUST = 0;
    public static final double RIGHT_ALIGN_ADJUST = 0;

    public static final int FRONT_LEFT_MODULE_STEER_ENCODER_ID = 10; // Set front left steer encoder ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER_ID = 11; // Set front right steer encoder ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER_ID = 12; // Set back left steer encoder ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER_ID = 13; // Set back right steer encoder ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 47; // Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 48; // Set front left module steer motor ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(306 + LEFT_ALIGN_ADJUST); // FIXME
                                                                                                          // Measure and
                                                                                                          // set front
    // left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 56; // Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 57; // Set front right steer motor ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(164 + RIGHT_ALIGN_ADJUST); // FIXME
                                                                                                            // Measure
                                                                                                            // and set
                                                                                                            // front
    // right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 50; // Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 51; // Set back left steer motor ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(123.5 + LEFT_ALIGN_ADJUST); // FIXME
                                                                                                           // Measure
                                                                                                           // and set
                                                                                                           // back
    // left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 53; // Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 54; // Set back right steer motor ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(65.75 + RIGHT_ALIGN_ADJUST); // FIXME
                                                                                                             // Measure
                                                                                                             // and set
                                                                                                             // back
    // right steer offset

    public static final int DRIVETRAIN_PIGEON_ID = 61; // Set Pigeon ID

    public static final double BAD_APRIL_TAG_ID = 2228;
    public static final double WAIT_TIME_AFTER_APRIL_TAG_DOCK_S = 0.3;

    // arm stuff
    public static final int ELEVATOR_MOTOR_ID = 58;
    public static final int ELEVATOR_LOWER_LIMIT_DIO = 1;
    public static final int ELEVATOR_UPPER_LIMIT_DIO = 0;
    public static final double ELEVATOR_MOTOR_SPEED = .1; // TODO - Need to determine speed for this motor
    public static final int ELEVATOR_ENCODER_COUNT_PER_INCH = 5023;
    public static final int ELEVATOR_ENCODER_SPOT_FLOOR = 0;
    public static final int ELEVATOR_ENCODER_SPOT_MIDDLE = 4;
    public static final int ELEVATOR_ENCODER_SPOT_TOP = 8;

    public static final int EXTENDO_MOTOR_ID = 59;
    public static final int EXTENDO_HOME_LIMIT_DIO = 3;
    public static final double EXTENDO_MOTOR_SPEED = .1; // TODO - Need to determine speed for this motor

    public static final int GRIPPER_PCM_PORT = 2;
    public static final int PCM_CAN_ID = 5;
    public static final int I2C_MUX_DIO = 2;
    public static final int ELEVATOR_DISTANCE_SENSOR_ADDRESS = 0x22;
    public static final int EXTENDO_ARM_DISTANCE_SENSOR_ADDRESS = 0x26;

    public enum PlacePosition {
        HighCone,
        MiddleCone,
        LowCone,
        HighCube,
        MiddleCube,
        LowCube
    }

    public enum PieceType {
        Cone,
        Cube
    }

    public enum ConeOffsetPosition {
        Left,
        Right
    }

    // Read left to right from corresponding driver station
    public enum AutoPosition {
        Position1,
        Position2,
        Position3
    }

    public static final double MAX_AUTO_VELOCITY = 4.0;
    public static final double MAX_AUTO_ACCELERATION = 3.0;

    public static final double GRID_STRAFE_DISTANCE = 55.88;
    public static final double SUBSTATION_STRAFE_DISTANCE = 55.88;
    public static final double NUDGE_STRAFE_DISTANCE = 3.0;
    public static final double STRAFE_SPEED = 0.2;
}
