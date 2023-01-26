package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
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
    private double m_armJoystick;
    private double m_strafeJoystick;

    public ButtonBoardSubsystem() {
        m_joystick1 = new Joystick(kJoystickChannel1);
        m_joystick2 = new Joystick(kJoystickChannel2);
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
        return new JoystickButton(m_joystick2, 12);
    }

    private JoystickButton getArmDownButton() {
        return new JoystickButton(m_joystick2, 12);
    }

    private JoystickButton getSubstationDockButton() {
        return new JoystickButton(m_joystick2, 12);
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

        System.out.println("Setting April Tag: " + m_aprilTagID);
    }

    public double getAprilTagID() {
        return m_aprilTagID;
    }

    @Override
    public void periodic() {
        if (getSubstationLeftRightToggleSwitch().getAsBoolean()) {
            m_substationShelfPosition = SubstationShelfPosition.Right;
        } else {
            m_substationShelfPosition = SubstationShelfPosition.Left;
        }

        if (getAutoManualToggleSwitch().getAsBoolean()) {
            m_operationMode = ButtonBoardOperationMode.Manual;
        } else {
            m_operationMode = ButtonBoardOperationMode.Auto;
        }

        if (getOpenCloseGripperToggleSwitch().getAsBoolean()) {
            m_gripperMode = GripperMode.Closed;
        } else {
            m_gripperMode = GripperMode.Open;
        }

        m_strafeJoystick = m_joystick2.getRawAxis(0);
        m_armJoystick = m_joystick2.getRawAxis(1);
    }

    public void configureButtonBindings() {

        // TODO - do we want to do something cool at each stage like with LEDs?
        // We could create multiple eventMaps
        HashMap<String, Command> eventMap = new HashMap<>();

        // !! Robot MUST BE ENABLED for these commands to work !!

        // Arm and Docking Buttons
        getSubstationDockButton().onTrue(new SequentialCommandGroup(new PrintCommand("Docking")));

        getArmUpButton().onTrue(new SequentialCommandGroup(new PrintCommand("Arm Up")));

        getArmDownButton().onTrue(new SequentialCommandGroup(new PrintCommand("Docking")));

        // Bottom Three Red Buttons used to place game piece
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

        // Middle Three Red Buttons used to place game piece
        getMiddleLeftConeButton().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Left Cone")));

        getMiddleCubeButton().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Cube")));

        getMiddleRightConeButton().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Right Cone")));

        // Top Three Red Buttons used to place game piece
        getLowLeftConeButton().onTrue(new SequentialCommandGroup(new PrintCommand("Low Left Cone")));

        getLowCubeButton().onTrue(new SequentialCommandGroup(new PrintCommand("Low Cube")));

        getLowRightConeButton().onTrue(new SequentialCommandGroup(new PrintCommand("Low Right Cone")));

        // Three Blue Buttons to set the position based off of Alliance setting
        getPosition1Button().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 1"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position1))));

        getPosition2Button().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 2"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position2))));

        getPosition3Button().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 3"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position3))));
    }
}