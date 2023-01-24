package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.DockWithAprilTagCommand;

public class ButtonBoardManager {

    private final int kJoystickChannel = 1;
    private Joystick m_joystick;
    private Constants.AutoPosition m_gridPosition = Constants.AutoPosition.Position1;
    private double m_aprilTagID = Constants.BAD_APRIL_TAG_ID;

    public ButtonBoardManager() {
        m_joystick = new Joystick(kJoystickChannel);
    }
    // public static Joystick m_buttonBoardManager = new Joystick(1);

    public JoystickButton getJoystickButton1() {
        return new JoystickButton(m_joystick, 1);
    }

    public JoystickButton getJoystickButton2() {
        return new JoystickButton(m_joystick, 2);
    }

    public JoystickButton getJoystickButton3() {
        return new JoystickButton(m_joystick, 3);
    }

    public JoystickButton getJoystickButton4() {
        return new JoystickButton(m_joystick, 4);
    }

    public JoystickButton getJoystickButton5() {
        return new JoystickButton(m_joystick, 5);
    }

    public JoystickButton getJoystickButton6() {
        return new JoystickButton(m_joystick, 6);
    }

    public JoystickButton getJoystickButton7() {
        return new JoystickButton(m_joystick, 7);
    }

    public JoystickButton getJoystickButton8() {
        return new JoystickButton(m_joystick, 8);
    }

    public JoystickButton getJoystickButton9() {
        return new JoystickButton(m_joystick, 9);
    }

    public JoystickButton getJoystickButton10() {
        return new JoystickButton(m_joystick, 10);
    }

    public JoystickButton getJoystickButton11() {
        return new JoystickButton(m_joystick, 11);
    }

    public JoystickButton getJoystickButton12() {
        return new JoystickButton(m_joystick, 12);
    }

    public void setPosition(Constants.AutoPosition position) {

        System.out.println("setPosition" + position);

        m_gridPosition = position;
    }

    public double getAprilTagID() {

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

        return m_aprilTagID;
    }

    public void configureButtonBindings() {

        // Bottom Three Red Buttons used to place game piece
        getJoystickButton1().onTrue(new SequentialCommandGroup(new PrintCommand("High Left Cone")));
        getJoystickButton2().onTrue(new SequentialCommandGroup(new PrintCommand("High Cube")));
        getJoystickButton3().onTrue(new SequentialCommandGroup(new PrintCommand("Hifh Right Cone")));

        // Middle Three Red Buttons used to place game piece
        getJoystickButton4().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Left Cone")));
        getJoystickButton5().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Cube")));
        getJoystickButton6().onTrue(new SequentialCommandGroup(new PrintCommand("Middle Right Cone")));

        // Top Three Red Buttons used to place game piece
        getJoystickButton7().onTrue(new SequentialCommandGroup(new PrintCommand("Low Left Cone")));
        getJoystickButton8().onTrue(new SequentialCommandGroup(new PrintCommand("Low Cube" )));
        getJoystickButton9().onTrue(new SequentialCommandGroup(new PrintCommand("Low Right Cone")));

        // Three Blue Buttons to set the position based of Alliance
        // MUST BE ENABLED
        getJoystickButton10().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 1"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position1)),
                new InstantCommand(() -> getAprilTagID())));
        getJoystickButton11().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 2"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position2)),
                new InstantCommand(() -> getAprilTagID())));
        getJoystickButton12().onTrue(new SequentialCommandGroup(new PrintCommand("Setting Position 3"),
                new InstantCommand(() -> setPosition(Constants.AutoPosition.Position3)),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(false)),
                new DockWithAprilTagCommand(false, true),
                new InstantCommand(() -> RobotContainer.getDrivetrainSubsystem().setPathPlannerDriving(true))));
    }
}