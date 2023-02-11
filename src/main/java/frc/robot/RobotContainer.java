// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoOneCommand;
import frc.robot.commands.AutoThreeCommand;
import frc.robot.commands.AutoTwoCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
import frc.robot.utils.ButtonBoardManager;
import frc.robot.utils.ShuffleboardManager;
import frc.robot.utils.AprilTagManager;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.DistanceSensorSubsystem;


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
    // private final static DrivetrainSubsystem m_drivetrainSubsystem = new
    // DrivetrainSubsystem();

    private final static XboxController m_controller = new XboxController(0);
    // private Joystick m_buttonBox2 = new Joystick(2);

    private final static AprilTagManager m_aprilTagManager = new AprilTagManager();

    private final static LEDStripSubsystem m_ledStripSubsystem = new LEDStripSubsystem();
    
    private final static DistanceSensorSubsystem m_distance_sensor_sensor_subsystem = new DistanceSensorSubsystem();

    private final static ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem(m_distance_sensor_sensor_subsystem);

    private final static ShuffleboardTab m_autoConfigTab = Shuffleboard.getTab("Auto Config");
    private final static ShuffleboardManager m_shuffleboardManager = new ShuffleboardManager(m_autoConfigTab);

    // private final static ButtonBoardManager m_buttonBoardManager = new ButtonBoardManager();

    private final static ExtendoSubsystem m_extendo_subsystem = new ExtendoSubsystem(m_distance_sensor_sensor_subsystem);

    private final static PneumaticSubsystem m_pneumatic_subsystem = new PneumaticSubsystem();



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
        // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        // m_drivetrainSubsystem,
        // () -> -modifyAxis(m_controller.getLeftY()) *
        // m_drivetrainSubsystem.getForwardAdjustment()
        // * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        // () -> -modifyAxis(m_controller.getLeftX()) *
        // m_drivetrainSubsystem.getSidewaysAdjustment()
        // * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        // () -> -modifyAxis(m_controller.getRightX()) *
        // m_drivetrainSubsystem.getRotationalAdjustment()
        // * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

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

        // Configure the XboxController buttons
        // new Trigger(m_controller::getBackButton) // Back button zeros the gyroscope
        // .onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope));
        // new Trigger(m_controller::getAButton) // TESTING a button raises elevator
        // .onTrue(new InstantCommand(m_elevatorSubsystem::raiseElevator));
        // new Trigger(m_controller::getBButton) // TESTING b button retracts elevator
        // .onTrue(new InstantCommand(m_elevatorSubsystem::lowerElevator));
        // new Trigger(m_controller::getXButton) // TESTING x button extends arm
        // .onTrue(new InstantCommand(m_elevatorSubsystem::goToFloor));
        // new Trigger(m_controller::getYButton) // TESTING y button retracts arm
        // .onTrue(new InstantCommand(m_elevatorSubsystem::goToTop));

        // Configure all the buttons and switches on the Custom Button Board
        new Trigger(m_controller::getXButton).onTrue(
                Commands.runOnce(() -> {
                    double current_val = m_elevatorSubsystem.getMeasurement();
                    System.out.println("X Button: current: " + current_val);
                    m_elevatorSubsystem.setElevatorPosition(30);
                },m_elevatorSubsystem));

        new Trigger(m_controller::getYButton).onTrue(
                Commands.runOnce(() -> {
                    double current_val = m_elevatorSubsystem.getMeasurement();
                    System.out.println("Y Button: current: " + current_val);
                    m_elevatorSubsystem.setElevatorPosition(0);
                }, m_elevatorSubsystem));

        new Trigger(m_controller::getAButton).onTrue(
                Commands.runOnce(() -> {
                    System.out.println("A Button: current: ");
                    m_extendo_subsystem.goToDisanceCM(78);
                }, m_elevatorSubsystem));

        new Trigger(m_controller::getBButton).onTrue(
                Commands.runOnce(() -> {
                    System.out.println("B Button: current: ");
                    m_extendo_subsystem.goToDisanceCM(22.5);
                }, m_elevatorSubsystem));

        // m_buttonBoardManager.configureButtonBindings();

        new Trigger(m_controller::getRightBumper).onTrue(
            new ArmCommand(m_extendo_subsystem, m_elevatorSubsystem, ArmCommand.Destination.bot)
        );

        new Trigger(m_controller::getLeftBumper).onTrue(
                Commands.runOnce(() -> {
                    m_pneumatic_subsystem.closeGripper();
                }, m_pneumatic_subsystem));

        // new Trigger(new JoystickButton(m_buttonBox2, 3)).onTrue(
        //             Commands.runOnce(() -> {
        //                 m_extendo_subsystem.goToDisanceCM(78);
        //                 //System.out.println("Right High");
        //             }));
        // new Trigger(new JoystickButton(m_buttonBox2, 6)).onTrue(
        //             Commands.runOnce(() -> {
        //                 m_extendo_subsystem.goToDisanceCM(38);
        //                 //System.out.println("Right Mid");
        //             }));
        // new Trigger(new JoystickButton(m_buttonBox2, 9)).onTrue(
        //             Commands.runOnce(() -> {
        //                 m_extendo_subsystem.goToDisanceCM(22.5);
        //                 System.out.println("Right Low");
        // //             }));
        
        // new Trigger(new JoystickButton(m_buttonBox2, 12)).onTrue(
        //             );
                

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
        if (getShuffleboardManager().getAutoPosition() == Constants.AutoPosition.Position1) {
            return new AutoOneCommand();
        } else if (getShuffleboardManager().getAutoPosition() == Constants.AutoPosition.Position2) {
            return new AutoTwoCommand();
        } else if (getShuffleboardManager().getAutoPosition() == Constants.AutoPosition.Position3) {
            return new AutoThreeCommand();
        } else {
            System.out.println("Invalid position received from Shufflboard");
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

    // public static DrivetrainSubsystem getDrivetrainSubsystem() {
    // return m_drivetrainSubsystem;
    // }

    public static AprilTagManager getAprilTagManager() {
        return m_aprilTagManager;
    }

    public static LEDStripSubsystem getLEDStripSubsystem() {
        return m_ledStripSubsystem;
    }

    public static ElevatorSubsystem getArmSubsystem() {
        return m_elevatorSubsystem;
    }

    public static ShuffleboardManager getShuffleboardManager() {
        return m_shuffleboardManager;
    }

    // public static ButtonBoardManager getButtonBoardManager() {
    //     return m_buttonBoardManager;
    // }

    public static XboxController getXboxController() {
        return m_controller;
    }
}
