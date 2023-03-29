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
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.712; // Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.810; // Measure and set wheelbase

    // Based on a 4" swerve wheel
    public final static double DRIVETRAIN_WHEEL_CIRCUMFERENCE_CM = 31.9278;

    // Falcon ticks per rotation is 2048 * SDS Mk4i Gear Ratio of 6.75:1
    // private final static double TICKS_PER_ROTATION = 2048.0 * 6.75; // 13824
    public final static double DRIVETRAIN_TICKS_PER_ROTATION = 12900.00;

    // CAN ID Assignments
    // Since CAN messages are prioritized based on CAN ID, we are setting the CAN

    public static final int FRONT_LEFT_MODULE_STEER_ENCODER_ID = 10; // Set front left steer encoder ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER_ID = 11; // Set front right steer encoder ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER_ID = 12; // Set back left steer encoder ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER_ID = 13; // Set back right steer encoder ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 47; // Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 48; // Set front left module steer motor ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(198);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 56; // Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 57; // Set front right steer motor ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(92);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 50; // Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 51; // Set back left steer motor ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(204);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 53; // Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 54; // Set back right steer motor ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(211); 
    public static final int DRIVETRAIN_PIGEON_ID = 61; // Set Pigeon ID

    // arm stuff

    public enum ArmDestination {
        shelf,
        high,
        middle,
        low,
        home,
        transit,
        cube,
        cone_floor
    }

    public static final int ELEVATOR_MOTOR_ID = 58;
    public static final int ELEVATOR_LOWER_LIMIT_DIO = 1;
    public static final int ELEVATOR_UPPER_LIMIT_DIO = 0;
    public static final double ELEVATOR_MOTOR_SPEED = 0.75; 

    public static final int EXTENDO_MOTOR_ID = 59;
    public static final int EXTENDO_HOME_LIMIT_DIO = 3;
    public static final double EXTENDO_MOTOR_SPEED = 0.75;

    public static final int GRIPPER_PCM_PORT = 0;
    public static final int ELEVATOR_BRAKE_PCM_PORT = 1;
    public static final int ARM_BRAKE_PCM_PORT = 2;
    public static final int LOW_SIDE_ANALOG_PORT = 0;
    public static final int HIGH_SIDE_ANALOG_PORT = 1;
    public static final int PCM_CAN_ID = 5;

    public static final int GRABBING_SENSOR_DIO = 4;

    // Read left to right from corresponding driver station
    public enum AutoPosition {
        Position1,
        Position2,
        Position2_Out,
        Position3,
        ConeOnly
    }

    public static final double MAX_AUTO_VELOCITY = 3.0;
    public static final double MAX_AUTO_ACCELERATION = 2.0;

    public static final double CAMERA_OFFSET_METERS = 0.14;
    public static final double GRID_BACK_OFF_DISTANCE_CM = 5.0;
    public static final double GRID_BACK_OFF_SPEED = 0.1;

    // PDH Constants
    public static final int WINCH_MOTOR_PDH_PORT = 14;
    public static final int ELEVATOR_MOTOR_PDH_PORT = 5;
}
