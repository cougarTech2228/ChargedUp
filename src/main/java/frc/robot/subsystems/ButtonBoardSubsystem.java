package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.DockWithAprilTagCommand;
import frc.robot.commands.FollowTrajectoryCommand;

public class ButtonBoardSubsystem extends SubsystemBase {

    private final int kJoystickChannel1 = 1;
    private final int kJoystickChannel2 = 2;
    private Joystick m_joystick1;
    private Joystick m_Joystick2;
    private Constants.AutoPosition m_gridPosition = Constants.AutoPosition.Position1;
    private double m_aprilTagID = Constants.BAD_APRIL_TAG_ID;

    public ButtonBoardSubsystem() {
        m_joystick1 = new Joystick(kJoystickChannel1);
        m_Joystick2 = new Joystick(kJoystickChannel2);
    }

    private JoystickButton getHighLeftCone() {
        return new JoystickButton(m_joystick1, 1);
    }

    private JoystickButton getHighCube() {
        return new JoystickButton(m_joystick1, 2);
    }

    private JoystickButton getHighRightCone() {
        return new JoystickButton(m_joystick1, 3);
    }

    private JoystickButton getMiddleLeftCone() {
        return new JoystickButton(m_joystick1, 4);
    }

    private JoystickButton getMiddleCube() {
        return new JoystickButton(m_joystick1, 5);
    }

    private JoystickButton getMiddleRightCone() {
        return new JoystickButton(m_joystick1, 6);
    }

    private JoystickButton getLowLeftCone() {
        return new JoystickButton(m_joystick1, 7);
    }

    private JoystickButton getLowCube() {
        return new JoystickButton(m_joystick1, 8);
    }

    private JoystickButton getLowRightCone() {
        return new JoystickButton(m_joystick1, 9);
    }

    private JoystickButton getPosition1() {
        return new JoystickButton(m_joystick1, 10);
    }

    private JoystickButton getPosition2() {
        return new JoystickButton(m_joystick1, 11);
    }

    private JoystickButton getPosition3() {
        return new JoystickButton(m_joystick1, 12);
    }

    private JoystickButton getArmUp() {
        return new JoystickButton(m_Joystick2, 12);
    }

    private JoystickButton getArmDown() {
        return new JoystickButton(m_Joystick2, 12);
    }

    private JoystickButton getSubstationDock() {
        return new JoystickButton(m_Joystick2, 12);
    }

    private JoystickButton getLRToggleSwitch() {
        return new JoystickButton(m_Joystick2, 1);
    }

    private JoystickButton getAutoManualToggleSwich() {
        return new JoystickButton(m_Joystick2, 3);
    }

    private JoystickButton getGripperToggleSwich() {
        return new JoystickButton(m_Joystick2, 6);
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
        // Monitor the current state of the Joystick and Toggle Switches here

    }

    public void configureButtonBindings() {

        // TODO - do we want to do something cool at each stage like with LEDs?
        // We could create multiple eventMaps
        HashMap<String, Command> eventMap = new HashMap<>();

        // !! Robot MUST BE ENABLED for these commands to work !!

        // Arm and Docking Buttons
        getSubstationDock().onTrue(new SequentialCommandGroup(new PrintCommand("Docking")));

        getArmUp().onTrue(new SequentialCommandGroup(new PrintCommand("Arm Up")));

        getArmDown().onTrue(new SequentialCommandGroup(new PrintCommand("Docking")));

        // Bottom Three Red Buttons used to place game piece
        getHighLeftCone().onTrue(new SequentialCommandGroup(new PrintCommand("High Left Cone"),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new DockWithAprilTagCommand(true, true, this),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(true)),
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(),
                        "strafe_right", eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new PrintCommand("Placing game piece")));

        getHighCube().onTrue(new SequentialCommandGroup(new PrintCommand("High Cube"),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new DockWithAprilTagCommand(true, true, this),
                new PrintCommand("Placing game piece")));

        getHighRightCone().onTrue(new SequentialCommandGroup(new PrintCommand("High Left Cone"),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new DockWithAprilTagCommand(true, true, this),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(true)),
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(),
                        "strafe_left", eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new PrintCommand("Placing game piece")));

        // Middle Three Red Buttons used to place game piece
        getMiddleLeftCone().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Left Cone")));

        getMiddleCube().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Cube")));

        getMiddleRightCone().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Right Cone")));

        // Top Three Red Buttons used to place game piece
        getLowLeftCone().onTrue(new SequentialCommandGroup(new PrintCommand("Low Left Cone")));

        getLowCube().onTrue(new SequentialCommandGroup(new PrintCommand("Low Cube")));

        getLowRightCone().onTrue(new SequentialCommandGroup(new PrintCommand("Low Right Cone")));

        // Three Blue Buttons to set the position based of Alliance
        getPosition1().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 1"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position1))));

        getPosition2().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 2"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position2))));

        getPosition3().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 3"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position3))));
    }
}