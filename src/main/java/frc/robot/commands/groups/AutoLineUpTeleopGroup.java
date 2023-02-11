package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavX;
import frc.robot.commands.auto.AutoLineUp;
import frc.robot.commands.teleop.AutoBalanceTeleop;
import frc.robot.commands.teleop.AutoDriveTeleop;

public class AutoLineUpTeleopGroup {
    public static Command get(Drivetrain drivetrain) {
        return new AutoLineUp(drivetrain);
    }
}
