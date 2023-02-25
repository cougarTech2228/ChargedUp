package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.ArmDestination;
import frc.robot.commands.BackOffCommand;
import frc.robot.commands.DockWithAprilTagCommand;
import frc.robot.commands.ParallelArmCommand;
import frc.robot.commands.SetArmHeightCommand;
import frc.robot.commands.SetArmReachCommand;
import frc.robot.commands.StrafeCommand;
import frc.robot.utils.AprilTagManager;
import frc.robot.utils.CT_LEDStrip.GlowColor;

public class ButtonBoardSubsystem extends SubsystemBase {

    private enum SubstationShelfPosition {
        Left,
        Right
    }

    private enum ButtonBoardOperationMode {
        Fine,
        Coarse
    }

    private final int kJoystickChannel1 = 1;
    private final int kJoystickChannel2 = 2;

    private Joystick m_joystick1;
    private Joystick m_joystick2;

    private Constants.AutoPosition m_gridPosition = Constants.AutoPosition.Position1;

    private double m_aprilTagID = Constants.BAD_APRIL_TAG_ID;

    private SubstationShelfPosition m_substationShelfPosition;
    private ButtonBoardOperationMode m_operationMode;

    private double m_armReachJoystick;
    private double m_strafeJoystick;

    private boolean m_strafeReset = true;
    private boolean m_armReachReset = true;

    private static ExtendoSubsystem m_extendoSubsystem;
    private static ElevatorSubsystem m_elevatorSubsystem;
    private static AprilTagManager m_aprilTagManager;
    private static LEDStripSubsystem m_ledStripSubsystem;
    private static PneumaticSubsystem m_pneumaticSubsystem;
    private static DrivetrainSubsystem m_drivetrainSubsystem;
    private static ShuffleboardSubsystem m_shuffleboardSubsystem;

    private static final double FINE_INCREMENTAL_ARM_HEIGHT_CHANGE_CM = 2.0;
    private static final double FINE_INCREMENTAL_ARM_REACH_CHANGE_CM = 5.0;

    private static final double COARSE_INCREMENTAL_ARM_HEIGHT_CHANGE_CM = 10.0;
    private static final double COARSE_INCREMENTAL_ARM_REACH_CHANGE_CM = 10.0;

    public ButtonBoardSubsystem(ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem,
            AprilTagManager aprilTagManager, LEDStripSubsystem ledStripSubsystem, PneumaticSubsystem pneumaticSubsystem,
            DrivetrainSubsystem drivetrainSubsystem, ShuffleboardSubsystem shuffleboardSubsystem) {
        m_joystick1 = new Joystick(kJoystickChannel1);
        m_joystick2 = new Joystick(kJoystickChannel2);

        m_elevatorSubsystem = elevatorSubsystem;
        m_extendoSubsystem = extendoSubsystem;
        m_aprilTagManager = aprilTagManager;
        m_ledStripSubsystem = ledStripSubsystem;
        m_pneumaticSubsystem = pneumaticSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_shuffleboardSubsystem = shuffleboardSubsystem;

        setSubstationShelfPosition();
        setOperationMode();
    }

    private JoystickButton getHighLeftConeButton() {
        return new JoystickButton(m_joystick1, 1);
    }

    private JoystickButton getHighCubeButton() {
        return new JoystickButton(m_joystick1, 2);
    }

    private JoystickButton getHighRightConeButton() {
        return new JoystickButton(m_joystick1, 3);
    }

    private JoystickButton getMiddleLeftConeButton() {
        return new JoystickButton(m_joystick1, 4);
    }

    private JoystickButton getMiddleCubeButton() {
        return new JoystickButton(m_joystick1, 5);
    }

    private JoystickButton getMiddleRightConeButton() {
        return new JoystickButton(m_joystick1, 6);
    }

    private JoystickButton getLowLeftConeButton() {
        return new JoystickButton(m_joystick1, 7);
    }

    private JoystickButton getLowCubeButton() {
        return new JoystickButton(m_joystick1, 8);
    }

    private JoystickButton getLowRightConeButton() {
        return new JoystickButton(m_joystick1, 9);
    }

    private JoystickButton getPosition1Button() {
        return new JoystickButton(m_joystick1, 10);
    }

    private JoystickButton getPosition2Button() {
        return new JoystickButton(m_joystick1, 11);
    }

    private JoystickButton getPosition3Button() {
        return new JoystickButton(m_joystick1, 12);
    }

    private JoystickButton getArmUpButton() {
        return new JoystickButton(m_joystick2, 4);
    }

    private JoystickButton getArmDownButton() {
        return new JoystickButton(m_joystick2, 5);
    }

    private JoystickButton getSubstationDockButton() {
        return new JoystickButton(m_joystick2, 2);
    }

    private JoystickButton getSubstationLeftRightToggleSwitch() {
        return new JoystickButton(m_joystick2, 1);
    }

    private JoystickButton getOperationToggleSwitch() {
        return new JoystickButton(m_joystick2, 3);
    }

    private JoystickButton getToggleGripperButton() {
        return new JoystickButton(m_joystick2, 6);
    }

    public boolean isAprilTagIDMatch() {
        if ((m_aprilTagID == m_aprilTagManager.getTagID())
                && (m_aprilTagID != Constants.BAD_APRIL_TAG_ID)) {
            m_ledStripSubsystem.glow(GlowColor.Green);
            return true;
        } else {
            m_ledStripSubsystem.glow(GlowColor.Red);
            return false;
        }
    }

    private void resetAprilTagID() {
        m_aprilTagID = Constants.BAD_APRIL_TAG_ID;
    }

    private void setPosition(Constants.AutoPosition position) {

        System.out.println("setPosition" + position);

        m_gridPosition = position;

        if (DriverStation.getAlliance() == Alliance.Blue) {
            if (m_gridPosition == Constants.AutoPosition.Position1) {
                m_aprilTagID = 6.0;
            } else if (m_gridPosition == Constants.AutoPosition.Position2) {
                m_aprilTagID = 7.0;
            } else if (m_gridPosition == Constants.AutoPosition.Position3) {
                m_aprilTagID = 8.0;
            } else {
                System.out.println("Alliance Position Set Error");
            }
        } else if (DriverStation.getAlliance() == Alliance.Red) {
            if (m_gridPosition == Constants.AutoPosition.Position1) {
                m_aprilTagID = 1.0;
            } else if (m_gridPosition == Constants.AutoPosition.Position2) {
                m_aprilTagID = 2.0;
            } else if (m_gridPosition == Constants.AutoPosition.Position3) {
                m_aprilTagID = 3.0;
            } else {
                System.out.println("Alliance Position Set Error");
            }
        } else {
            System.out.println("Alliance Color Error");
        }

        System.out.println("Setting Position April Tag: " + m_aprilTagID);
    }

    private void setDockingStation() {

        if (DriverStation.getAlliance() == Alliance.Blue) {
            m_aprilTagID = 4.0;
        } else if (DriverStation.getAlliance() == Alliance.Red) {
            m_aprilTagID = 5.0;
        } else {
            System.out.println("Alliance Color Error");
        }

        System.out.println("Setting Docking Station April Tag: " + m_aprilTagID);
    }

    public double getAprilTagID() {
        return m_aprilTagID;
    }

    private boolean isFineOperationMode() {
        return (m_operationMode == ButtonBoardOperationMode.Fine);
    }

    private boolean isLeftDockingStation() {
        return (m_substationShelfPosition == SubstationShelfPosition.Left);
    }

    private void setSubstationShelfPosition() {
        if (getSubstationLeftRightToggleSwitch().getAsBoolean()) {
            m_substationShelfPosition = SubstationShelfPosition.Right;
        } else {
            m_substationShelfPosition = SubstationShelfPosition.Left;
        }
    }

    private void setOperationMode() {
        if (getOperationToggleSwitch().getAsBoolean()) {
            m_operationMode = ButtonBoardOperationMode.Fine;
        } else {
            m_operationMode = ButtonBoardOperationMode.Coarse;
        }
    }

    @Override
    public void periodic() {

        setSubstationShelfPosition();
        setOperationMode();

        // Handle the joystick strafing input
        m_strafeJoystick = m_joystick2.getRawAxis(0);

        // We only want to allow the user to strafe right or left one step
        // at time such that the joystick has to return to its center
        // position before another strafe command is issued. This should
        // stop a situation where multiple stafe commands are issued if the
        // user were to hold the joystick in the extreme right or left
        // position.
        if (m_strafeReset) {

            if (m_strafeJoystick == 1.0) { // Right
                new StrafeCommand(Constants.NUDGE_STRAFE_DISTANCE, -Constants.STRAFE_SPEED, false,
                        m_drivetrainSubsystem, m_aprilTagManager).schedule();
                m_strafeReset = false;
            } else if (m_strafeJoystick == -1.0) { // Left
                new ScheduleCommand(new StrafeCommand(Constants.NUDGE_STRAFE_DISTANCE, Constants.STRAFE_SPEED, false,
                        m_drivetrainSubsystem, m_aprilTagManager))
                        .schedule();
                m_strafeReset = false;
            }
        }

        // The joystick value reports 1.0 and -1.0, but it never gets exactly
        // zero so we need to perform this check to reset the ability to send
        // another strafe command.
        if ((m_strafeJoystick < 1.0) && (m_strafeJoystick > -1.0)) {
            m_strafeReset = true;
        }

        // Handle the joystick arm reach input
        m_armReachJoystick = m_joystick2.getRawAxis(1);

        // We only want to allow the user to adjust the arm reach one step
        // at time such that the joystick has to return to its center
        // position before another arm reach command is issued. This should
        // stop a situation where multiple arm reach commands are issued if
        // the user were to hold the joystick in the extreme up or down
        // position.
        if (m_armReachReset) {

            if (m_armReachJoystick == 1.0) { // Arm Extend
                if (isFineOperationMode()) {
                    m_extendoSubsystem.goToDistanceCM(
                            m_extendoSubsystem.getCurrentArmReachCm() + FINE_INCREMENTAL_ARM_REACH_CHANGE_CM);
                } else {
                    m_extendoSubsystem.goToDistanceCM(
                            m_extendoSubsystem.getCurrentArmReachCm() + COARSE_INCREMENTAL_ARM_REACH_CHANGE_CM);
                }
                m_armReachReset = false;
            } else if (m_armReachJoystick == -1.0) { // Arm Retract
                if (isFineOperationMode()) {
                    m_extendoSubsystem.goToDistanceCM(
                            m_extendoSubsystem.getCurrentArmReachCm() - FINE_INCREMENTAL_ARM_REACH_CHANGE_CM);
                } else {
                    m_extendoSubsystem.goToDistanceCM(
                            m_extendoSubsystem.getCurrentArmReachCm() - COARSE_INCREMENTAL_ARM_REACH_CHANGE_CM);
                }
                m_armReachReset = false;
            }
        }

        // The joystick value reports 1.0 and -1.0, but it never gets exactly
        // zero so we need to perform this check to reset the ability to send
        // another arm reach command.
        if ((m_armReachJoystick < 1.0) && (m_armReachJoystick > -1.0)) {
            m_armReachReset = true;
        }

        m_shuffleboardSubsystem.getTargetIDEntry().setDouble(getAprilTagID());
    }

    public void configureButtonBindings() {

        // !! Robot MUST BE ENABLED for these commands to work !!

        // **********************************
        // Gripper Button Handling
        // **********************************
        getToggleGripperButton().onTrue(
                new ConditionalCommand(
                        // True command
                        new InstantCommand(() -> m_pneumaticSubsystem.toggleGripper()),
                        // False command
                        new PrintCommand("You must be in manual mode to toggle the gripper"),

                        // variable
                        this::isFineOperationMode));

        // **********************************
        // Docking Station Button Handling
        // **********************************
        getSubstationDockButton().onTrue(
                new SequentialCommandGroup(
                        new PrintCommand("Docking with Substation"),
                        new InstantCommand(() -> setDockingStation()),
                        new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                        new ParallelCommandGroup(new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.shelf),
                                new SetArmReachCommand(m_extendoSubsystem, ArmDestination.tight)),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new DockWithAprilTagCommand(false, this, m_aprilTagManager,
                                                m_drivetrainSubsystem),
                                        new WaitCommand(Constants.WAIT_TIME_AFTER_APRIL_TAG_DOCK_S),
                                        new ConditionalCommand(
                                                new StrafeCommand(Constants.SUBSTATION_STRAFE_DISTANCE,
                                                        -Constants.STRAFE_SPEED,
                                                        true, m_drivetrainSubsystem, m_aprilTagManager),
                                                new StrafeCommand(Constants.SUBSTATION_STRAFE_DISTANCE,
                                                        Constants.STRAFE_SPEED,
                                                        true, m_drivetrainSubsystem, m_aprilTagManager),
                                                this::isLeftDockingStation)/*
                                                                            * ,
                                                                            * new ParallelArmCommand(m_extendoSubsystem,
                                                                            * m_elevatorSubsystem,
                                                                            * ArmDestination.shelf),
                                                                            * new InstantCommand(() ->
                                                                            * m_pneumaticSubsystem.closeGripper())
                                                                            */),
                                new PrintCommand("April Tag Not Detected"), () -> isAprilTagIDMatch())));

        // **********************************
        // Arm Button Handling
        // **********************************
        getArmUpButton().onTrue(
                new ConditionalCommand(
                        // True command
                        Commands.runOnce(() -> {
                            if (isFineOperationMode()) {
                                m_elevatorSubsystem.setElevatorPosition(m_elevatorSubsystem.getMeasurement()
                                        + FINE_INCREMENTAL_ARM_HEIGHT_CHANGE_CM);
                            }
                        }),
                        // False command
                        Commands.runOnce(() -> {
                            if (isFineOperationMode()) {
                                m_elevatorSubsystem.setElevatorPosition(m_elevatorSubsystem.getMeasurement()
                                        + COARSE_INCREMENTAL_ARM_HEIGHT_CHANGE_CM);
                            }
                        }),

                        // variable
                        this::isFineOperationMode));

        getArmDownButton().onTrue(
                new ConditionalCommand(
                        // True command
                        Commands.runOnce(() -> {
                            if (isFineOperationMode()) {
                                m_elevatorSubsystem.setElevatorPosition(m_elevatorSubsystem.getMeasurement()
                                        - FINE_INCREMENTAL_ARM_HEIGHT_CHANGE_CM);
                            }
                        }),
                        // False command
                        Commands.runOnce(() -> {
                            if (isFineOperationMode()) {
                                m_elevatorSubsystem.setElevatorPosition(m_elevatorSubsystem.getMeasurement()
                                        - COARSE_INCREMENTAL_ARM_HEIGHT_CHANGE_CM);
                            }
                        }),

                        // variable
                        this::isFineOperationMode));

        // **********************************
        // Placing Position Button Handling
        // **********************************

        // Place high game pieces
        getHighLeftConeButton().onTrue(
                new SequentialCommandGroup(
                        new PrintCommand("High Left Cone"),
                        new ParallelCommandGroup(new SetArmHeightCommand(m_elevatorSubsystem,
                                ArmDestination.high),
                                new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home)),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new DockWithAprilTagCommand(true, this, m_aprilTagManager,
                                                m_drivetrainSubsystem),
                                        new WaitCommand(Constants.WAIT_TIME_AFTER_APRIL_TAG_DOCK_S),
                                        new StrafeCommand(
                                                Constants.GRID_STRAFE_DISTANCE,
                                                Constants.STRAFE_SPEED,
                                                true, m_drivetrainSubsystem, m_aprilTagManager),
                                        new ParallelArmCommand(m_extendoSubsystem, m_elevatorSubsystem,
                                                ArmDestination.high),
                                        /*
                                         * new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                                         * new SetArmReachCommand(m_extendoSubsystem, ArmDestination.tight),
                                         * new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                                         * new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home),
                                         */
                                        new InstantCommand(() -> resetAprilTagID())),
                                new PrintCommand("April Tag Not Detected"), () -> isAprilTagIDMatch())));

        getHighCubeButton().onTrue(new SequentialCommandGroup(
                new PrintCommand("High Cube"),
                new ParallelCommandGroup(new SetArmHeightCommand(m_elevatorSubsystem,
                        ArmDestination.high),
                        new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home)),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new DockWithAprilTagCommand(true, this, m_aprilTagManager,
                                        m_drivetrainSubsystem),
                                new ParallelArmCommand(m_extendoSubsystem, m_elevatorSubsystem,
                                        ArmDestination.high),
                                /*
                                 * new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                                 * new SetArmReachCommand(m_extendoSubsystem, ArmDestination.tight),
                                 * new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                                 * new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home),
                                 */
                                new InstantCommand(() -> resetAprilTagID())),
                        new PrintCommand("April Tag Not Detected"), () -> isAprilTagIDMatch())));

        getHighRightConeButton().onTrue(
                new SequentialCommandGroup(
                        new PrintCommand("High Right Cone"),
                        new ParallelCommandGroup(new SetArmHeightCommand(m_elevatorSubsystem,
                                ArmDestination.high),
                                new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home)),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new DockWithAprilTagCommand(true, this, m_aprilTagManager,
                                                m_drivetrainSubsystem),
                                        new WaitCommand(Constants.WAIT_TIME_AFTER_APRIL_TAG_DOCK_S),
                                        new StrafeCommand(Constants.GRID_STRAFE_DISTANCE, -Constants.STRAFE_SPEED,
                                                true, m_drivetrainSubsystem, m_aprilTagManager),
                                        new ParallelArmCommand(m_extendoSubsystem, m_elevatorSubsystem,
                                                ArmDestination.high),
                                        /*
                                         * new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                                         * new SetArmReachCommand(m_extendoSubsystem, ArmDestination.tight),
                                         * new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                                         * new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home),
                                         */
                                        new InstantCommand(() -> resetAprilTagID())),
                                new PrintCommand("April Tag Not Detected"), () -> isAprilTagIDMatch())));

        // Place middle game pieces
        getMiddleLeftConeButton().onTrue(
                new SequentialCommandGroup(
                        new PrintCommand("Middle Left Cone"),
                        new ParallelCommandGroup(new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.middle),
                                new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home)),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new DockWithAprilTagCommand(true, this, m_aprilTagManager,
                                                m_drivetrainSubsystem),
                                        new WaitCommand(Constants.WAIT_TIME_AFTER_APRIL_TAG_DOCK_S),
                                        new StrafeCommand(
                                                Constants.GRID_STRAFE_DISTANCE,
                                                Constants.STRAFE_SPEED,
                                                true, m_drivetrainSubsystem, m_aprilTagManager),
                                        new ParallelArmCommand(m_extendoSubsystem, m_elevatorSubsystem,
                                                ArmDestination.middle),
                                        /*
                                         * new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                                         * new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home),
                                         * new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                                         * new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home),
                                         */
                                        new InstantCommand(() -> resetAprilTagID())),
                                new PrintCommand("April Tag Not Detected"), () -> isAprilTagIDMatch())));

        getMiddleCubeButton().onTrue(
                new SequentialCommandGroup(
                        new PrintCommand("Middle Cube"),
                        new ParallelCommandGroup(new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.middle),
                                new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home)),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new DockWithAprilTagCommand(true, this, m_aprilTagManager,
                                                m_drivetrainSubsystem),
                                        new ParallelArmCommand(m_extendoSubsystem, m_elevatorSubsystem,
                                                ArmDestination.middle),
                                        /*
                                         * new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                                         * new SetArmReachCommand(m_extendoSubsystem, ArmDestination.tight),
                                         * new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                                         * new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home),
                                         */
                                        new InstantCommand(() -> resetAprilTagID())),
                                new PrintCommand("April Tag Not Detected"), () -> isAprilTagIDMatch())));

        getMiddleRightConeButton().onTrue(
                new SequentialCommandGroup(
                        new PrintCommand("Middle Right Cone"),
                        new ParallelCommandGroup(
                                new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.middle),
                                new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home)),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new DockWithAprilTagCommand(true, this, m_aprilTagManager,
                                                m_drivetrainSubsystem),
                                        new WaitCommand(Constants.WAIT_TIME_AFTER_APRIL_TAG_DOCK_S),
                                        new StrafeCommand(
                                                Constants.GRID_STRAFE_DISTANCE,
                                                -Constants.STRAFE_SPEED,
                                                true, m_drivetrainSubsystem, m_aprilTagManager),
                                        new ParallelArmCommand(m_extendoSubsystem, m_elevatorSubsystem,
                                                ArmDestination.middle),
                                        /*
                                         * new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                                         * new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home),
                                         * new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                                         * new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home),
                                         */
                                        new InstantCommand(() -> resetAprilTagID())),
                                new PrintCommand("April Tag Not Detected"), () -> isAprilTagIDMatch())));

        // Place low game pieces
        getLowLeftConeButton().onTrue(
                new SequentialCommandGroup(
                        new PrintCommand("Low Left Cone"),
                        new ParallelCommandGroup(
                                new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.middle),
                                new SetArmReachCommand(m_extendoSubsystem, ArmDestination.low)),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new DockWithAprilTagCommand(true, this, m_aprilTagManager,
                                                m_drivetrainSubsystem),
                                        new WaitCommand(Constants.WAIT_TIME_AFTER_APRIL_TAG_DOCK_S),
                                        new StrafeCommand(
                                                Constants.GRID_STRAFE_DISTANCE,
                                                Constants.STRAFE_SPEED,
                                                true, m_drivetrainSubsystem, m_aprilTagManager),
                                        new SetArmReachCommand(m_extendoSubsystem, ArmDestination.tight),
                                        new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.low),
                                        new SetArmReachCommand(m_extendoSubsystem, ArmDestination.low),
                                        /*
                                         * new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                                         * new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home),
                                         * new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                                         * new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home),
                                         */
                                        new InstantCommand(() -> resetAprilTagID())),
                                new PrintCommand("April Tag Not Detected"), () -> isAprilTagIDMatch())));

        getLowCubeButton().onTrue(
                new SequentialCommandGroup(
                        new PrintCommand("Low Cube"),
                        new ParallelCommandGroup(
                                new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.middle),
                                new SetArmReachCommand(m_extendoSubsystem, ArmDestination.low)),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new DockWithAprilTagCommand(true, this, m_aprilTagManager,
                                                m_drivetrainSubsystem),
                                        new SetArmReachCommand(m_extendoSubsystem, ArmDestination.tight),
                                        new BackOffCommand(Constants.GRID_BACK_OFF_DISTANCE_CM,
                                                Constants.GRID_BACK_OFF_SPEED, m_drivetrainSubsystem),
                                        new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.low),
                                        new BackOffCommand(Constants.GRID_BACK_OFF_DISTANCE_CM,
                                                -Constants.GRID_BACK_OFF_SPEED, m_drivetrainSubsystem),
                                        new SetArmReachCommand(m_extendoSubsystem, ArmDestination.low),
                                        /*
                                         * new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                                         * new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home),
                                         * new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                                         * new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home),
                                         */
                                        new InstantCommand(() -> resetAprilTagID())),
                                new PrintCommand("April Tag Not Detected"), () -> isAprilTagIDMatch())));

        getLowRightConeButton().onTrue(
                new SequentialCommandGroup(
                        new PrintCommand("Low Right Cone"),
                        new ParallelCommandGroup(
                                new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.middle),
                                new SetArmReachCommand(m_extendoSubsystem, ArmDestination.low)),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new DockWithAprilTagCommand(true, this, m_aprilTagManager,
                                                m_drivetrainSubsystem),
                                        new WaitCommand(Constants.WAIT_TIME_AFTER_APRIL_TAG_DOCK_S),
                                        new StrafeCommand(
                                                Constants.GRID_STRAFE_DISTANCE,
                                                -Constants.STRAFE_SPEED,
                                                true, m_drivetrainSubsystem, m_aprilTagManager),
                                        new SetArmReachCommand(m_extendoSubsystem, ArmDestination.tight),
                                        new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.low),
                                        new SetArmReachCommand(m_extendoSubsystem, ArmDestination.low),
                                        /*
                                         * new InstantCommand(() -> m_pneumaticSubsystem.openGripper()),
                                         * new SetArmReachCommand(m_extendoSubsystem, ArmDestination.home),
                                         * new InstantCommand(() -> m_pneumaticSubsystem.closeGripper()),
                                         * new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.home),
                                         */
                                        new InstantCommand(() -> resetAprilTagID())),
                                new PrintCommand("April Tag Not Detected"), () -> isAprilTagIDMatch())));

        // Set the desired grid position where the game piece will
        getPosition1Button().onTrue(
                new SequentialCommandGroup(
                        new PrintCommand("Setting Position 1"),
                        new InstantCommand(() -> setPosition(Constants.AutoPosition.Position1))));

        getPosition2Button().onTrue(
                new SequentialCommandGroup(
                        new PrintCommand("Setting Position 2"),
                        new InstantCommand(() -> setPosition(Constants.AutoPosition.Position2))));

        getPosition3Button().onTrue(
                new SequentialCommandGroup(
                        new PrintCommand("Setting Position 3"),
                        new InstantCommand(() -> setPosition(Constants.AutoPosition.Position3))));
    }
}