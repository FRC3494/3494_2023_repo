package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class AutoLineUpTeleopGroup {
    public static Command get(Drivetrain drivetrain) {
        return new PrintCommand("hello world");//AutoLineUp(drivetrain);
    }
}
