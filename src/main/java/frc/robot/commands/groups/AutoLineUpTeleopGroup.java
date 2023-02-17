package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.auto.AutoLineUp;

public class AutoLineUpTeleopGroup {
    public static Command get(Drivetrain drivetrain) {
        return new AutoLineUp(drivetrain);
    }
}
