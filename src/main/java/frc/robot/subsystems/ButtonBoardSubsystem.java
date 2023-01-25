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

    private final int kJoystickChannel = 1;
    private Joystick m_joystick;
    private Constants.AutoPosition m_gridPosition = Constants.AutoPosition.Position1;
    private double m_aprilTagID = Constants.BAD_APRIL_TAG_ID;

    public ButtonBoardSubsystem() {
        m_joystick = new Joystick(kJoystickChannel);
    }

    private JoystickButton getJoystickButton1() {
        return new JoystickButton(m_joystick, 1);
    }

    private JoystickButton getJoystickButton2() {
        return new JoystickButton(m_joystick, 2);
    }

    private JoystickButton getJoystickButton3() {
        return new JoystickButton(m_joystick, 3);
    }

    private JoystickButton getJoystickButton4() {
        return new JoystickButton(m_joystick, 4);
    }

    private JoystickButton getJoystickButton5() {
        return new JoystickButton(m_joystick, 5);
    }

    private JoystickButton getJoystickButton6() {
        return new JoystickButton(m_joystick, 6);
    }

    private JoystickButton getJoystickButton7() {
        return new JoystickButton(m_joystick, 7);
    }

    private JoystickButton getJoystickButton8() {
        return new JoystickButton(m_joystick, 8);
    }

    private JoystickButton getJoystickButton9() {
        return new JoystickButton(m_joystick, 9);
    }

    private JoystickButton getJoystickButton10() {
        return new JoystickButton(m_joystick, 10);
    }

    private JoystickButton getJoystickButton11() {
        return new JoystickButton(m_joystick, 11);
    }

    private JoystickButton getJoystickButton12() {
        return new JoystickButton(m_joystick, 12);
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

        // Bottom Three Red Buttons used to place game piece
        getJoystickButton1().onTrue(new SequentialCommandGroup(new PrintCommand("High Left Cone"),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new DockWithAprilTagCommand(true, false, this),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(true)),
                new FollowTrajectoryCommand(RobotContainer.getDrivetrainSubsystem(),
                        "strafe_right", eventMap,
                        Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION, true),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new PrintCommand("Placing game piece")));

        getJoystickButton2().onTrue(new SequentialCommandGroup(new PrintCommand("High Cube")));

        getJoystickButton3().onTrue(new SequentialCommandGroup(new PrintCommand("High Right Cone")));

        // Middle Three Red Buttons used to place game piece
        getJoystickButton4().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Left Cone")));

        getJoystickButton5().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Cube")));

        getJoystickButton6().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Right Cone")));

        // Top Three Red Buttons used to place game piece
        getJoystickButton7().onTrue(new SequentialCommandGroup(new PrintCommand("Low Left Cone")));

        getJoystickButton8().onTrue(new SequentialCommandGroup(new PrintCommand("Low Cube")));

        getJoystickButton9().onTrue(new SequentialCommandGroup(new PrintCommand("Low Right Cone")));

        // Three Blue Buttons to set the position based of Alliance
        getJoystickButton10().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 1"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position1))));

        getJoystickButton11().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 2"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position2))));

        getJoystickButton12().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 3"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position3))));
    }
}