package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.auto.AutoBalanceReverse;
import frc.robot.commands.auto.AutoDrive;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoBalanceReverseGroup {
    public static Command get(Drivetrain drivetrain) {
        return new AutoDrive(drivetrain, Constants.Commands.AutoBalance.FAST_POWER, 0, 0, false)
                .until(() -> Math.abs(NavX.getPitch()) >= Constants.Commands.AutoBalance.PEAK_ANGLE)
                .andThen(new AutoBalanceReverse(drivetrain));
    }
}
