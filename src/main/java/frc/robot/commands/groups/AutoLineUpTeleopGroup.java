package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.Drivetrain;

public class AutoLineUpTeleopGroup {
    public static Pose2d get(Drivetrain drivetrain, Field2d f) {
        return new Pose2d(new Translation2d(10, 10), new Rotation2d(0));
        //FollowPath(drivetrain, drivetrain.getPathToTag(), f);
    }
}
