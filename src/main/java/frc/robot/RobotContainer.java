// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoOneCommand;
import frc.robot.commands.AutoThreeCommand;
import frc.robot.commands.AutoTwoCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DockWithAprilTagCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
import frc.robot.utils.ShuffleboardManager;
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

    private final static XboxController m_controller = new XboxController(0);

    private final static AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();

    private final static LEDStripSubsystem m_ledStripSubsystem = new LEDStripSubsystem();

    private final static ShuffleboardTab m_autoConfigTab = Shuffleboard.getTab("Auto Config");
    private final static ShuffleboardManager m_shuffleboardManager = new ShuffleboardManager(m_autoConfigTab);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // This has to be called first to setup the Shuffleboard controls which
        // are used in the configureButtonBindings method.
        m_shuffleboardManager.configureShuffleboard();

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
                () -> -modifyAxis(m_controller.getRightX()) * m_drivetrainSubsystem.getRotationalAdjustment()
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

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new InstantCommand(() -> {
            m_ledStripSubsystem.rainbow();
        }));

        // Back button zeros the gyroscope
        new Trigger(m_controller::getBackButton)
                // No requirements because we don't need to interrupt anything
                .onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope));

        new Trigger(m_controller::getBButton)
                .onTrue(new InstantCommand(m_drivetrainSubsystem::stopMotors, m_drivetrainSubsystem));

        // TODO - REMOVE - Temporary bindings for debug purposes
         new Trigger(m_controller::getYButton)
                 .onTrue(new AutoThreeCommand());

        // new Trigger(m_controller::getAButton)
        //         .onTrue(new SequentialCommandGroup(
        //                 new InstantCommand(m_drivetrainSubsystem::setPathPlannerDriving),
        //                 new InstantCommand(m_drivetrainSubsystem::setMotorsToBrake),
        //                 new FollowTrajectoryCommand(m_drivetrainSubsystem, "auto3_out", eventMap, 4.0, 3.0, true),
        //                 new FollowTrajectoryCommand(m_drivetrainSubsystem, "auto3_back", eventMap, 4.0, 3.0, true),
        //                 new InstantCommand(m_drivetrainSubsystem::setNotPathPlannerDriving),
        //                 new DockWithAprilTagCommand(false)
        //                 ));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        switch (DriverStation.getLocation()) {
            case 1:
                return new AutoOneCommand();
            case 2:
                return new AutoTwoCommand();
            case 3:
                return new AutoThreeCommand();
            default:
                System.out.println("Invalid position received from DriverStation");
                return null;
        }
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

    public static LEDStripSubsystem getLEDStripSubsystem() {
        return m_ledStripSubsystem;
    }

    public static ShuffleboardManager getShuffleboardManager() {
        return m_shuffleboardManager;
    }

    public static XboxController getXboxController() {
        return m_controller;
    }
}
