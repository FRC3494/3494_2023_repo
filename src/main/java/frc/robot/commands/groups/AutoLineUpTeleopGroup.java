package frc.robot.commands.groups;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.FollowPath;
import frc.robot.subsystems.Drivetrain;

public class AutoLineUpTeleopGroup {
    public static Command get(Drivetrain drivetrain, Field2d f) {
        //return new Pose2d(new Translation2d(10, 10), new Rotation2d(0));\
        PathPlannerTrajectory traj = PathPlanner.generatePath(
            new PathConstraints(4, 3), 
            new PathPoint(new Translation2d(drivetrain.getPose().getX(), drivetrain.getPose().getY()), drivetrain.getPose().getRotation()), // position, heading
            new PathPoint(new Translation2d(1.6, 1), Rotation2d.fromDegrees(0)) // position, heading
        );
        return new FollowPath(drivetrain, traj, f);
    }
}
