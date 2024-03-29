// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmDestination;
import frc.robot.commands.AutoConeOnlyCommand;
import frc.robot.commands.AutoOneCommand;
import frc.robot.commands.AutoThreeCommand;
import frc.robot.commands.AutoTwoCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.SetArmHeightCommand;
import frc.robot.commands.SetArmReachCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.LEDStripSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.subsystems.ButtonBoardSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

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

    // The Shuffleboard stuff needs to be called early because some of the data
    // updates by other subsystems in their periodics make calls to Shuffleboard.
    private final static ShuffleboardTab m_chargedUpTab = Shuffleboard.getTab("ChargedUp!");
    private final static ShuffleboardSubsystem m_shuffleboardSubsystem = new ShuffleboardSubsystem(m_chargedUpTab);

    private final static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

    private final static XboxController m_controller = new XboxController(0);

    private final static LEDStripSubsystem m_ledStripSubsystem = new LEDStripSubsystem();

    private final static PneumaticSubsystem m_pneumaticSubsystem = new PneumaticSubsystem(m_ledStripSubsystem);
    
    private final static ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem(m_pneumaticSubsystem);

    private final static ExtendoSubsystem m_extendoSubsystem = new ExtendoSubsystem(m_pneumaticSubsystem);

    private final static ButtonBoardSubsystem m_buttonBoardSubsystem = new ButtonBoardSubsystem(m_elevatorSubsystem,
            m_extendoSubsystem, m_pneumaticSubsystem, m_drivetrainSubsystem);

    private final static PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        DriverStation.silenceJoystickConnectionWarning(true);

        // This has to be called first to setup the Shuffleboard controls which
        // are used in the configureButtonBindings method.
        m_shuffleboardSubsystem.configureShuffleboard();

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

        new Trigger(m_controller::getXButton)
        .onTrue(new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()));

        new Trigger(m_controller::getBackButton)
                .onTrue(new InstantCommand(() -> m_drivetrainSubsystem.reverseGyroscope()));

        new Trigger(m_controller::getBButton)
        .onTrue(new InstantCommand(() -> cancelAllCommands()));

        new Trigger(m_controller::getLeftBumperPressed)
                .onTrue(new InstantCommand(() -> m_drivetrainSubsystem.setBoostMode(true)));

        new Trigger(m_controller::getLeftBumperReleased)
                .onTrue(new InstantCommand(() -> m_drivetrainSubsystem.setBoostMode(false)));
        
        new Trigger(m_controller::getRightBumperPressed).onTrue( new InstantCommand(() -> {
            m_drivetrainSubsystem.lockWheels();
        }));

        new Trigger(m_controller::getAButton)
                .onTrue(
                    new InstantCommand(() -> {
                        m_pneumaticSubsystem.toggleArmBrake();
                    }));
                        // new SequentialCommandGroup(
                        //         new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                        //         new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home),
                        //         new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home)));

        // PID TUNING DEBUG ---------------------
        // new Trigger(m_controller::getYButton)
        // .onTrue(new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.high));

        // new Trigger(m_controller::getBButton)
        // .onTrue(new SetArmReachCommand(m_extendoSubsystem, ArmDestination.high));

        // new Trigger(m_controller::getXButton)
        // .onTrue(new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home));

        // PID TUNING DEBUG ---------------------

        // Configure all the buttons and switches on the Custom Button Board
        m_buttonBoardSubsystem.configureButtonBindings();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public static Command getAutonomousCommand() {
        if (m_shuffleboardSubsystem.getAutoPosition() == Constants.AutoPosition.Position1) {
            return new AutoOneCommand(m_elevatorSubsystem, m_extendoSubsystem, m_drivetrainSubsystem,
                    m_pneumaticSubsystem);
        } else if (m_shuffleboardSubsystem.getAutoPosition() == Constants.AutoPosition.Position2) {
            return new AutoTwoCommand(false, m_elevatorSubsystem, m_extendoSubsystem, m_drivetrainSubsystem,
                    m_pneumaticSubsystem, m_ledStripSubsystem);
        } else if (m_shuffleboardSubsystem.getAutoPosition() == Constants.AutoPosition.Position2_Out) {
            return new AutoTwoCommand(true, m_elevatorSubsystem, m_extendoSubsystem, m_drivetrainSubsystem,
                    m_pneumaticSubsystem, m_ledStripSubsystem);
        } else if (m_shuffleboardSubsystem.getAutoPosition() == Constants.AutoPosition.Position3) {
            return new AutoThreeCommand(m_elevatorSubsystem, m_extendoSubsystem, m_drivetrainSubsystem,
                    m_pneumaticSubsystem);
        } else if (m_shuffleboardSubsystem.getAutoPosition() == Constants.AutoPosition.ConeOnly) {
            return new AutoConeOnlyCommand(m_elevatorSubsystem, m_extendoSubsystem, m_drivetrainSubsystem, m_pneumaticSubsystem);
        } else {
            System.out.println("Invalid position received from Shufflboard");
            return null;
        }
    }

    public static void cancelAllCommands() {
        CommandScheduler.getInstance().cancelAll();
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

    public static boolean isEndgame(){
        if(DriverStation.isTeleop() && DriverStation.getMatchTime() <= 30){
            return true;
        } else {
            return false;
        }
    }

    public static DrivetrainSubsystem getDrivetrainSubsystem() {
        return m_drivetrainSubsystem;
    }

    public static LEDStripSubsystem getLEDStripSubsystem() {
        return m_ledStripSubsystem;
    }

    public static ButtonBoardSubsystem getButtonBoardSubsystem() {
        return m_buttonBoardSubsystem;
    }

    public static PneumaticSubsystem getPneumaticSubsystem() {
        return m_pneumaticSubsystem;
    }

    public static ElevatorSubsystem getElevatorSubsystem() {
        return m_elevatorSubsystem;
    }

    public static ExtendoSubsystem getExtendoSubsystem() {
        return m_extendoSubsystem;
    }

    public static ShuffleboardSubsystem getShuffleboardSubsystem(){
        return m_shuffleboardSubsystem;
    }

    public static PowerDistribution getPDH() {
        return m_pdh;
    }

}
