package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavX;
import frc.robot.commands.teleop.AutoBalanceTeleop;
import frc.robot.commands.teleop.AutoDriveTeleop;

public class AutoBalanceTeleopGroup {
    public static Command get(Drivetrain drivetrain) {
        return new AutoDriveTeleop(drivetrain, Constants.Commands.AutoBalance.DRIVE_POWER, 0, 0, true)
				.until(() -> Math.abs(NavX.getPitch()) >= Constants.Commands.AutoBalance.TRIGGER_ANGLE)
				.andThen(new AutoBalanceTeleop(drivetrain));
    }
}
