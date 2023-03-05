package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.teleop.AutoBalanceTeleop;
import frc.robot.commands.teleop.AutoDriveTeleop;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavX;

public class AutoBalanceTeleopGroup {
    public static Command get(Drivetrain drivetrain) {
        return new AutoDriveTeleop(drivetrain, -Constants.Commands.AutoBalance.FAST_POWER, 0, 0, false)
                .until(() -> Math.abs(NavX.getRoll()) >= Constants.Commands.AutoBalance.TRIGGER_ANGLE)
                .andThen(new AutoBalanceTeleop(drivetrain));
    }
    /*
     * l
     * public static Command get(Drivetrain drivetrain) {
     * ///drivetrain.resetOdometry(new Pose2d(new Translation2d(10, 10), new
     * Rotation2d(0)));
     * return new PrintCommand("Current Odo:" + drivetrain.getPose().getX() + ":" +
     * drivetrain.getPose().getY());
     * //FollowPath(drivetrain, drivetrain.getPathToTag(), f);
     * }
     */

}
