package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoDrive;
import frc.robot.commands.teleop.TeleopDriveInterruptor;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoBalanceTeleopGroup {
    public static Command get(Drivetrain drivetrain) {
        return new TeleopDriveInterruptor().deadlineWith(new AutoDrive(drivetrain, 0, -Constants.Commands.AutoBalance.FAST_POWER, 0, false)
                .until(() -> Math.abs(NavX.getRoll()) >= Constants.Commands.AutoBalance.PEAK_ANGLE)
                .andThen(new AutoBalance(drivetrain)));
    }
}
