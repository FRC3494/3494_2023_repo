package frc.robot.commands.groups;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.FollowPath;
import frc.robot.commands.teleop.TeleopDriveInterruptor;
import frc.robot.subsystems.Drivetrain;

public class AutoLineUpTeleopGroup {
    public static Command get(Drivetrain drivetrain, Field2d field2d) {
        //return new Pose2d(new Translation2d(10, 10), new Rotation2d(0));\
        PathPlannerTrajectory trajectoryToAprilTag = PathPlanner.generatePath(
            new PathConstraints(2.75, 1.25), 
            new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(180), drivetrain.getPose().getRotation()), // position, heading
            new PathPoint(new Translation2d(2.05, 2.1), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180)) // position, heading

        );
        //CUBE MID   : 2.05, 2.75
        //CONE RIGHT : 2.05, 2.1
        return new TeleopDriveInterruptor().deadlineWith(new FollowPath(drivetrain, trajectoryToAprilTag, field2d, false));
    }
}
