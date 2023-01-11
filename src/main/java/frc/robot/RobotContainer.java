// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DockWithAprilTagCommand;
import frc.robot.commands.DockWithAprilTagPIDCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
import frc.robot.subsystems.AprilTagPVSubsystem;
import frc.robot.subsystems.AprilTagSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

    private final XboxController m_controller = new XboxController(0);

    private final static AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();

    private final static AprilTagPVSubsystem m_aprilTagPVSubsystem = new AprilTagPVSubsystem();

    private final static LEDStripSubsystem m_ledStripSubsystem = new LEDStripSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Set up the default command for the drivetrain.
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        // isFieldOriented (true or false)
        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
                m_drivetrainSubsystem,
                () -> -modifyAxis(m_controller.getLeftY()) * m_drivetrainSubsystem.getForwardAdjustment()
                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(m_controller.getLeftX()) * m_drivetrainSubsystem.getSidewaysAdjustment()
                        * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(m_controller.getRightX()) * m_drivetrainSubsystem.getRotationalAdjustmennt()
                        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Back button zeros the gyroscope
        new Trigger(m_controller::getBackButton)
                // No requirements because we don't need to interrupt anything
                .onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope));

        // new Trigger(m_controller::getYButton)
        //         .onTrue(new DockWithAprilTagPIDCommand(m_drivetrainSubsystem, m_aprilTagSubsystem, 13.0));
        
        new Trigger(m_controller::getYButton)
                .onTrue(new DockWithAprilTagCommand(m_drivetrainSubsystem, m_aprilTagPVSubsystem, 1.0));

        new Trigger(m_controller::getBButton)
                .onTrue(new InstantCommand(m_drivetrainSubsystem::stopMotors, m_drivetrainSubsystem));

        new Trigger(m_controller::getAButton)
                .onTrue(new SequentialCommandGroup(new InstantCommand(m_drivetrainSubsystem::setDoingTeleOpAuto),
                        new InstantCommand(m_drivetrainSubsystem::setMotorsToBrake),
                        new FollowTrajectoryCommand(m_drivetrainSubsystem,
                                PathPlanner.loadPath("Path2", new PathConstraints(3, 1.5)), true)
                                .andThen(() -> m_drivetrainSubsystem.setNotDoingTeleOpAuto(), m_drivetrainSubsystem)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new InstantCommand();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis. Decreases sensitivity at lower speeds
        value = Math.copySign(value * value, value);

        return value;
    }

    public static DrivetrainSubsystem getDrivetrainSubsystem() {
        return m_drivetrainSubsystem;
    }

    public static AprilTagSubsystem getAprilTagSubsystem() {
        return m_aprilTagSubsystem;
    }

    public static AprilTagPVSubsystem getAprilTagPVSubsystem() {
        return m_aprilTagPVSubsystem;
    }

    public static LEDStripSubsystem getLEDStripSubsystem() {
        return m_ledStripSubsystem;
    }
}
