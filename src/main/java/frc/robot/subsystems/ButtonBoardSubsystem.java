package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.DockWithAprilTagCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.SetArmHeightCommand;
import frc.robot.commands.SetArmReachCommand;

public class ButtonBoardSubsystem extends SubsystemBase {

    private enum SubstationShelfPosition {
        Left,
        Right
    }

    private enum ButtonBoardOperationMode {
        Manual,
        Auto
    }

    private enum GripperMode {
        Open,
        Closed
    }

    private final int kJoystickChannel1 = 1;
    private final int kJoystickChannel2 = 2;

    private Joystick m_joystick1;
    private Joystick m_joystick2;

    private Constants.AutoPosition m_gridPosition = Constants.AutoPosition.Position1;

    private double m_aprilTagID = Constants.BAD_APRIL_TAG_ID;

    private SubstationShelfPosition m_substationShelfPosition;
    private ButtonBoardOperationMode m_operationMode;
    private GripperMode m_gripperMode;

    private double m_armReachJoystick;
    private double m_strafeJoystick;

    private boolean m_strafeReset = true;
    private boolean m_armReachReset = true;

    private static final double INCREMENTAL_ARM_HEIGHT_CHANGE_CM = 2.0;
    private static final double INCREMENTAL_ARM_REACH_CHANGE_CM = 2.0;

    private HashMap<String, Command> m_eventMap = new HashMap<>();

    public ButtonBoardSubsystem() {
        m_joystick1 = new Joystick(kJoystickChannel1);
        m_joystick2 = new Joystick(kJoystickChannel2);

        setSubstationShelfPosition();
        setAutoManualOperationMode();
        setGripperMode();
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

    private JoystickButton getAutoManualToggleSwitch() {
        return new JoystickButton(m_joystick2, 3);
    }

    private JoystickButton getOpenCloseGripperToggleSwitch() {
        return new JoystickButton(m_joystick2, 6);
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

    private boolean isManualOperationMode() {
        return (m_operationMode == ButtonBoardOperationMode.Manual);
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

    private void setAutoManualOperationMode() {
        if (getAutoManualToggleSwitch().getAsBoolean()) {
            m_operationMode = ButtonBoardOperationMode.Manual;
        } else {
            m_operationMode = ButtonBoardOperationMode.Auto;
        }
    }

    private void setGripperMode() {
        if (getOpenCloseGripperToggleSwitch().getAsBoolean()) {
            m_gripperMode = GripperMode.Closed;
        } else {
            m_gripperMode = GripperMode.Open;
        }
    }

    @Override
    public void periodic() {

        setSubstationShelfPosition();
        setAutoManualOperationMode();

        // Handle the Gripper Open/Closed toggle switch
        if (getOpenCloseGripperToggleSwitch().getAsBoolean()) { // Switch is in Closed position
            // Only allow changes in Manual mode
            if (isManualOperationMode()) {
                // If we're already closed don't do anything, otherwise close the gripper
                if (m_gripperMode != GripperMode.Closed) {
                    new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(false));
                    m_gripperMode = GripperMode.Closed;
                }
            }
        } else { // Switch is in Open position
            // Only allow changes in Manual mode
            if (isManualOperationMode()) {
                // If we're already open don't do anything, otherwise open the gripper
                if (m_gripperMode != GripperMode.Open) {
                    new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true));
                    m_gripperMode = GripperMode.Open;
                }
            }
        }

        // Handle the joystick strafing input
        m_strafeJoystick = m_joystick2.getRawAxis(0);

        // We only want to allow the user to strafe right or left one step
        // at time such that the joystick has to return to its center
        // position before another strafe command is issued. This should
        // stop a situation where multiple stafe commands are issued if the
        // user were to hold the joystick in the extreme right or left
        // position.
        if (m_strafeReset && isManualOperationMode()) {
            m_strafeReset = false;

            if (m_strafeJoystick == 1.0) {
                new SequentialCommandGroup(
                        new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(true)),
                        new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setMotorsToBrake),
                        new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "nudge_right", m_eventMap,
                                Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                        new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)));
            } else if (m_strafeJoystick == -1.0) {
                new SequentialCommandGroup(
                        new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(true)),
                        new InstantCommand(RobotContainer.getDrivetrainSubsystem()::setMotorsToBrake),
                        new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(), "nudge_left", m_eventMap,
                                Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                        new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)));
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
        if (m_armReachReset && isManualOperationMode()) {
            m_armReachReset = false;

            if (m_armReachJoystick == 1.0) {
                new SetArmReachCommand(
                        RobotContainer.getArmSubsystem().getCurrentArmReachCm() + INCREMENTAL_ARM_REACH_CHANGE_CM);
            } else if (m_strafeJoystick == -1.0) {
                new SetArmReachCommand(
                        RobotContainer.getArmSubsystem().getCurrentArmReachCm() - INCREMENTAL_ARM_REACH_CHANGE_CM);
            }
        }

        // The joystick value reports 1.0 and -1.0, but it never gets exactly
        // zero so we need to perform this check to reset the ability to send
        // another arm reach command.
        if ((m_armReachJoystick < 1.0) && (m_armReachJoystick > -1.0)) {
            m_armReachReset = true;
        }

        RobotContainer.getShuffleboardManager().getTargetIDEntry().setDouble(getAprilTagID());
    }

    public void configureButtonBindings() {

        // TODO - do we want to do something cool at each stage like with LEDs?
        // We could create multiple eventMaps
        HashMap<String, Command> eventMap = new HashMap<>();

        // !! Robot MUST BE ENABLED for these commands to work !!

        // **********************************
        // Docking Station Button Handling
        // **********************************
        getSubstationDockButton().onTrue(new SequentialCommandGroup(new PrintCommand("Docking with Substation"),
                new InstantCommand(() -> setDockingStation()),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new DockWithAprilTagCommand(true, true, this),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(true)),
                new ConditionalCommand(new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(),
                        "strafe_left", eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                        new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(),
                                "strafe_right", eventMap,
                                Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                        this::isLeftDockingStation),
                new PrintCommand("TODO - Pick game piece off of shelf")));

        // **********************************
        // Arm Button Handling
        // **********************************
        getArmUpButton().onTrue(new ConditionalCommand(
                new SetArmHeightCommand(
                        RobotContainer.getArmSubsystem().getCurrentArmHeightCm() + INCREMENTAL_ARM_HEIGHT_CHANGE_CM),
                new PrintCommand("You must be in manual mode to move the arm up"),
                this::isManualOperationMode));

        getArmDownButton().onTrue(new ConditionalCommand(
                new SetArmHeightCommand(
                        RobotContainer.getArmSubsystem().getCurrentArmHeightCm() - INCREMENTAL_ARM_HEIGHT_CHANGE_CM),
                new PrintCommand("You must be in manual mode to move the arm down"),
                this::isManualOperationMode));

        // **********************************
        // Placing Position Button Handling
        // **********************************

        // Place high game pieces
        getHighLeftConeButton().onTrue(new SequentialCommandGroup(new PrintCommand("High Left Cone"),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new DockWithAprilTagCommand(true, true, this),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(true)),
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(),
                        "strafe_right", eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new ParallelCommandGroup(new SetArmHeightCommand(Constants.ARM_HIGH_CONE_HEIGHT_CM),
                        new SetArmReachCommand(Constants.ARM_HIGH_CONE_REACH_CM)),
                new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true))));

        getHighCubeButton().onTrue(new SequentialCommandGroup(new PrintCommand("High Cube"),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new DockWithAprilTagCommand(true, true, this),
                new ParallelCommandGroup(new SetArmHeightCommand(Constants.ARM_HIGH_CUBE_HEIGHT_CM),
                        new SetArmReachCommand(Constants.ARM_HIGH_CUBE_REACH_CM)),
                new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true))));

        getHighRightConeButton().onTrue(new SequentialCommandGroup(new PrintCommand("High Right Cone"),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new DockWithAprilTagCommand(true, true, this),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(true)),
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(),
                        "strafe_left", eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new ParallelCommandGroup(new SetArmHeightCommand(Constants.ARM_HIGH_CONE_HEIGHT_CM),
                        new SetArmReachCommand(Constants.ARM_HIGH_CONE_REACH_CM)),
                new InstantCommand(() -> RobotContainer.getArmSubsystem().setGripperOpen(true))));

        // Place middle game pieces
        getMiddleLeftConeButton().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Left Cone")));

        getMiddleCubeButton().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Cube")));

        getMiddleRightConeButton().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Right Cone")));

        // Place low game pieces
        getLowLeftConeButton().onTrue(new SequentialCommandGroup(new PrintCommand("Low Left Cone")));

        getLowCubeButton().onTrue(new SequentialCommandGroup(new PrintCommand("Low Cube")));

        getLowRightConeButton().onTrue(new SequentialCommandGroup(new PrintCommand("Low Right Cone")));

        // Set the desired grid position where the game piece will
        getPosition1Button().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 1"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position1))));

        getPosition2Button().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 2"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position2))));

        getPosition3Button().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 3"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position3))));
    }
}