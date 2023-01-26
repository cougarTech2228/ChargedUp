package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class HomeArmCommand extends ParallelCommandGroup {

    public HomeArmCommand() {

        addCommands(new SetArmHeightCommand(0.0),
                new SetArmReachCommand(0.0));
    }
}
