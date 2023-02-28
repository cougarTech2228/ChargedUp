package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.ArmDestination;
import frc.robot.commands.DriveFwdRevCommand;
import frc.robot.commands.ParallelArmCommand;
import frc.robot.commands.RotateBotCommand;
import frc.robot.commands.SetArmHeightCommand;
import frc.robot.commands.SetArmReachCommand;
import frc.robot.commands.StrafeCommand;

public class ButtonBoardSubsystem extends SubsystemBase {

    private enum ButtonBoardOperationMode {
        Fine,
        Coarse
    }

    private final int kJoystickChannel1 = 1;
    private final int kJoystickChannel2 = 2;

    private Joystick m_joystick1;
    private Joystick m_joystick2;

    private ButtonBoardOperationMode m_operationMode;

    private double m_fwdRevJoystick;
    private double m_strafeJoystick;

    private boolean m_strafeReset = true;
    private boolean m_fwdRevReset = true;

    private static ExtendoSubsystem m_extendoSubsystem;
    private static ElevatorSubsystem m_elevatorSubsystem;
    private static PneumaticSubsystem m_pneumaticSubsystem;
    private static DrivetrainSubsystem m_drivetrainSubsystem;

    private static final double FINE_INCREMENTAL_ARM_HEIGHT_CHANGE_CM = 2.0;
    private static final double COARSE_INCREMENTAL_ARM_HEIGHT_CHANGE_CM = 10.0;

    private static final double FINE_INCREMENTAL_ARM_REACH_CHANGE_CM = 6.0;
    private static final double COARSE_INCREMENTAL_ARM_REACH_CHANGE_CM = 10.0;

    private static final double FINE_DRIVE_DISTANCE_CM = 5.0;
    private static final double COARSE_DRIVE_DISTANCE_CM = 10.0;
    private static final double DRIVE_SPEED = 0.1;

    private static final double FINE_TURN_DEGREES = 2.0;
    private static final double COARSE_TURN_DEGREES = 6.0;
    private static final double ANGULAR_VELOCITY = 0.1;

    public ButtonBoardSubsystem(ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem,
            PneumaticSubsystem pneumaticSubsystem,
            DrivetrainSubsystem drivetrainSubsystem) {
        m_joystick1 = new Joystick(kJoystickChannel1);
        m_joystick2 = new Joystick(kJoystickChannel2);

        m_elevatorSubsystem = elevatorSubsystem;
        m_extendoSubsystem = extendoSubsystem;
        m_pneumaticSubsystem = pneumaticSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
    }

    // Joystick #1 Buttons

    private JoystickButton getArmShelfButton() {
        return new JoystickButton(m_joystick1, 1);
    }

    private JoystickButton getArmTransitButton() {
        return new JoystickButton(m_joystick1, 2);
    }

    private JoystickButton getArmHomeButton() {
        return new JoystickButton(m_joystick1, 3);
    }

    private JoystickButton getArmLowButton() {
        return new JoystickButton(m_joystick1, 4);
    }

    private JoystickButton getArmTBD1Button() {
        return new JoystickButton(m_joystick1, 5);
    }

    private JoystickButton getArmMiddleButton() {
        return new JoystickButton(m_joystick1, 6);
    }

    private JoystickButton getArmTBD2Button() {
        return new JoystickButton(m_joystick1, 7);
    }

    private JoystickButton getArmHighButton() {
        return new JoystickButton(m_joystick1, 8);
    }

    // Joystick #2 Buttons

    private JoystickButton getRotateLeftButton() {
        return new JoystickButton(m_joystick2, 1);
    }

    private JoystickButton getRotateRightButton() {
        return new JoystickButton(m_joystick2, 2);
    }

    private JoystickButton getArmUpButton() {
        return new JoystickButton(m_joystick2, 3);
    }

    private JoystickButton getArmRetractButton() {
        return new JoystickButton(m_joystick2, 4);
    }

    private JoystickButton getArmDownButton() {
        return new JoystickButton(m_joystick2, 5);
    }

    private JoystickButton getArmExtendButton() {
        return new JoystickButton(m_joystick2, 6);
    }

    private JoystickButton getOperationToggleSwitch() {
        return new JoystickButton(m_joystick2, 7);
    }

    private JoystickButton getToggleGripperButton() {
        return new JoystickButton(m_joystick2, 8);
    }

    private boolean isFineOperationMode() {
        return (m_operationMode == ButtonBoardOperationMode.Fine);
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
                new StrafeCommand(Constants.NUDGE_STRAFE_DISTANCE, -Constants.STRAFE_SPEED,
                        m_drivetrainSubsystem).schedule();
                m_strafeReset = false;
            } else if (m_strafeJoystick == -1.0) { // Left
                new ScheduleCommand(new StrafeCommand(Constants.NUDGE_STRAFE_DISTANCE, Constants.STRAFE_SPEED,
                        m_drivetrainSubsystem))
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

        // Handle the joystick forward/reverse input
        m_fwdRevJoystick = m_joystick2.getRawAxis(1);

        // We only want to allow the user to adjust the forward and reverse
        // position one step at a time and not continuously drive.
        if (m_fwdRevReset) {

            if (m_fwdRevJoystick == 1.0) { // Reverse
                if (isFineOperationMode()) {
                    new DriveFwdRevCommand(FINE_DRIVE_DISTANCE_CM,
                            DRIVE_SPEED, m_drivetrainSubsystem);
                } else {
                    new DriveFwdRevCommand(COARSE_DRIVE_DISTANCE_CM,
                            DRIVE_SPEED, m_drivetrainSubsystem);
                }
                m_fwdRevReset = false;
            } else if (m_fwdRevJoystick == -1.0) { // Forward
                if (isFineOperationMode()) {
                    new DriveFwdRevCommand(FINE_DRIVE_DISTANCE_CM,
                            -DRIVE_SPEED, m_drivetrainSubsystem);
                } else {
                    new DriveFwdRevCommand(COARSE_DRIVE_DISTANCE_CM,
                            -DRIVE_SPEED, m_drivetrainSubsystem);
                }
                m_fwdRevReset = false;
            }
        }

        // The joystick value reports 1.0 and -1.0, but it never gets exactly
        // zero so we need to perform this check to reset the ability to send
        // another forward/reverse command.
        if ((m_fwdRevJoystick < 1.0) && (m_fwdRevJoystick > -1.0)) {
            m_fwdRevReset = true;
        }
    }

    public void configureButtonBindings() {

        // !! Robot MUST BE ENABLED for these commands to work !!

        // **********************************
        // Gripper Button Handling
        // **********************************
        getToggleGripperButton().onTrue(
                new InstantCommand(() -> m_pneumaticSubsystem.toggleGripper()));

        // **********************************
        // Arm Button Handling
        // **********************************
        getArmUpButton().onTrue(
                new ConditionalCommand(
                        // True command
                        Commands.runOnce(() -> {
                            m_elevatorSubsystem.setElevatorPosition(m_elevatorSubsystem.getMeasurement()
                                    + FINE_INCREMENTAL_ARM_HEIGHT_CHANGE_CM);
                        }),
                        // False command
                        Commands.runOnce(() -> {
                            m_elevatorSubsystem.setElevatorPosition(m_elevatorSubsystem.getMeasurement()
                                    + COARSE_INCREMENTAL_ARM_HEIGHT_CHANGE_CM);
                        }),

                        // variable
                        this::isFineOperationMode));

        getArmDownButton().onTrue(
                new ConditionalCommand(
                        // True command
                        Commands.runOnce(() -> {
                            m_elevatorSubsystem.setElevatorPosition(m_elevatorSubsystem.getMeasurement()
                                    - FINE_INCREMENTAL_ARM_HEIGHT_CHANGE_CM);
                        }),
                        // False command
                        Commands.runOnce(() -> {
                            m_elevatorSubsystem.setElevatorPosition(m_elevatorSubsystem.getMeasurement()
                                    - COARSE_INCREMENTAL_ARM_HEIGHT_CHANGE_CM);
                        }),

                        // variable
                        this::isFineOperationMode));

        getArmExtendButton().onTrue(
                new ConditionalCommand(
                        // True command
                        Commands.runOnce(() -> {
                            m_extendoSubsystem.goToDistanceCM(
                                    m_extendoSubsystem.getCurrentArmReachCm() + FINE_INCREMENTAL_ARM_REACH_CHANGE_CM);
                        }),
                        // False command
                        Commands.runOnce(() -> {
                            m_extendoSubsystem.goToDistanceCM(
                                    m_extendoSubsystem.getCurrentArmReachCm() + COARSE_INCREMENTAL_ARM_REACH_CHANGE_CM);
                        }),

                        // variable
                        this::isFineOperationMode));

        getArmRetractButton().onTrue(
                new ConditionalCommand(
                        // True command
                        Commands.runOnce(() -> {
                            m_extendoSubsystem.goToDistanceCM(
                                    m_extendoSubsystem.getCurrentArmReachCm() - FINE_INCREMENTAL_ARM_REACH_CHANGE_CM);
                        }),
                        // False command
                        Commands.runOnce(() -> {
                            m_extendoSubsystem.goToDistanceCM(
                                    m_extendoSubsystem.getCurrentArmReachCm() - COARSE_INCREMENTAL_ARM_REACH_CHANGE_CM);
                        }),

                        // variable
                        this::isFineOperationMode));

        getArmShelfButton().onTrue(
                new ParallelArmCommand(m_extendoSubsystem, m_elevatorSubsystem, ArmDestination.shelf));

        getArmTransitButton().onTrue(
                new SequentialCommandGroup(
                        new SetArmReachCommand(m_extendoSubsystem, ArmDestination.tight),
                        new SetArmHeightCommand(m_elevatorSubsystem, ArmDestination.preloaded_cone)));

        getArmHomeButton().onTrue(
                new ParallelArmCommand(m_extendoSubsystem, m_elevatorSubsystem, ArmDestination.home));

        getArmLowButton().onTrue(
                new ParallelArmCommand(m_extendoSubsystem, m_elevatorSubsystem, ArmDestination.low));

        getArmMiddleButton().onTrue(
                new ParallelArmCommand(m_extendoSubsystem, m_elevatorSubsystem, ArmDestination.middle));

        getArmHighButton().onTrue(
                new ParallelArmCommand(m_extendoSubsystem, m_elevatorSubsystem, ArmDestination.high));

        // **********************************
        // Robot Button Handling
        // **********************************
        getRotateLeftButton().onTrue(
                new ConditionalCommand(
                        // True command
                        Commands.runOnce(() -> {
                            new RotateBotCommand(FINE_TURN_DEGREES, ANGULAR_VELOCITY, m_drivetrainSubsystem);
                        }),
                        // False command
                        Commands.runOnce(() -> {
                            new RotateBotCommand(COARSE_TURN_DEGREES, ANGULAR_VELOCITY, m_drivetrainSubsystem);
                        }),

                        // variable
                        this::isFineOperationMode));

        getRotateRightButton().onTrue(
                new ConditionalCommand(
                        // True command
                        Commands.runOnce(() -> {
                            new RotateBotCommand(-FINE_TURN_DEGREES, ANGULAR_VELOCITY, m_drivetrainSubsystem);
                        }),
                        // False command
                        Commands.runOnce(() -> {
                            new RotateBotCommand(-COARSE_TURN_DEGREES, ANGULAR_VELOCITY, m_drivetrainSubsystem);
                        }),

                        // variable
                        this::isFineOperationMode));
    }
}