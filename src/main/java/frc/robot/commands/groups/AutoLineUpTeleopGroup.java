package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavX;
import frc.robot.commands.auto.AutoLineUp;
import frc.robot.commands.auto.FollowPath;
import frc.robot.commands.teleop.AutoBalanceTeleop;
import frc.robot.commands.teleop.AutoDriveTeleop;

public class AutoLineUpTeleopGroup {
    public static Command get(Drivetrain drivetrain, Field2d f) {
        //drivetrain.resetOdometry(new Pose2d());
        return new FollowPath(drivetrain, drivetrain.getPathToTag(), f);
    }
}
