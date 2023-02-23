package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavX;
import frc.robot.commands.teleop.AutoBalanceTeleop;
import frc.robot.commands.teleop.AutoDriveTeleop;

public class AutoBalanceTeleopGroup {
    /*public static Command get(Drivetrain drivetrain) {
        return new AutoDriveTeleop(drivetrain, Constants.Commands.AutoBalance.DRIVE_POWER, 0, 0, true)
				.until(() -> Math.abs(NavX.getPitch()) >= Constants.Commands.AutoBalance.TRIGGER_ANGLE)
				.andThen(new AutoBalanceTeleop(drivetrain));
    }*/
    public static Command get(Drivetrain drivetrain) {
        ///drivetrain.resetOdometry(new Pose2d(new Translation2d(10, 10), new Rotation2d(0)));
        return new PrintCommand("Current Odo:" + drivetrain.getPose().getX() + ":" + drivetrain.getPose().getY());
        //FollowPath(drivetrain, drivetrain.getPathToTag(), f);
    }

}
