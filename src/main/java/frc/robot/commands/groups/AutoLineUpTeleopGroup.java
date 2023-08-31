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
import frc.robot.subsystems.drivetrain.DriveLocation;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoLineUpTeleopGroup {
    public static Command get(Drivetrain drivetrain, Field2d field2d) {
        // return new Pose2d(new Translation2d(10, 10), new Rotation2d(0));\
        PathPlannerTrajectory trajectoryToAprilTag = PathPlanner.generatePath(
                new PathConstraints(2.75, 1.25),
                new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(0),
                        drivetrain.getPose().getRotation()), // position, heading
                new PathPoint(new Translation2d(2.05, 2.1), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position,
                                                                                                                  // heading

        );
        // CUBE MID : 2.05, 2.75
        // CONE RIGHT : 2.05, 2.1

        // Estimated Positions: (The 2.05 is an estimate)
        /*
         * ALL ON BLUE
         * ON RED< ANGLE IS FLIPPED AND X IS ~14.5
         * Left-Station left-Cone (2.05, 5.00)
         * Left-Station Mid-Cube (2.05, 4.45)
         * Left-Station right-Cone (2.05, 3.90)
         * 
         * Mid-Station left-Cone (2.05, 3.30)
         * Mid-Station Mid-Cube (2.05, 2.75)
         * Mid-Station right-Cone (2.05, 2.20)
         * 
         * Right-Station left-Cone (2.05, 1.6)
         * Right-Station Mid-Cube (2.05,1.1)
         * Right-Station right-Cone (2.05,0.5)
         * 
         * Blue Double-Sub-RIGHT (15.75, 6)
         * Blue Double-Sub-LEFT (15.75, 7.30)
         * BLUE SINGLE (14, 7.5)
         * 
         * RED Double-Sub-RIGHT (0.85, 6)
         * RED Double-Sub-LEFT (0.8, 7.30)
         */
        return new TeleopDriveInterruptor()
                .deadlineWith(new FollowPath(drivetrain, trajectoryToAprilTag, field2d, false));
    }

    public static Command go(Drivetrain drivetrain, Field2d field2d, DriveLocation location) {
        PathPoint endPoint = new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(180),
                drivetrain.getPose().getRotation());
        ;
        switch (location) {
            case SingleSubstation:
                endPoint = new PathPoint(new Translation2d(14, 7.5), Rotation2d.fromDegrees(90),
                        Rotation2d.fromDegrees(180));
                break;
            case DoubleSubstationLeft:
                endPoint = new PathPoint(new Translation2d(15.75, 7.30), Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(180));
                break;
            case DoubleSubstationRight:
                endPoint = new PathPoint(new Translation2d(15.75, 6), Rotation2d.fromDegrees(0),
                        Rotation2d.fromDegrees(180));
                break;
            // ---------
            case LeftConeLeftGrid:
                endPoint = new PathPoint(new Translation2d(2.05, 5.00), Rotation2d.fromDegrees(180),
                        Rotation2d.fromDegrees(180));
                break;
            case MiddleCubeLeftGrid:
                endPoint = new PathPoint(new Translation2d(2.05, 4.45), Rotation2d.fromDegrees(180),
                        Rotation2d.fromDegrees(180));
                break;
            case RightConeLeftGrid:
                endPoint = new PathPoint(new Translation2d(2.05, 3.90), Rotation2d.fromDegrees(180),
                        Rotation2d.fromDegrees(180));
                break;
            // ---------
            case LeftConeMiddleGrid:
                endPoint = new PathPoint(new Translation2d(2.05, 3.30), Rotation2d.fromDegrees(180),
                        Rotation2d.fromDegrees(180));
                break;
            case MiddleCubeMiddleGrid:
                endPoint = new PathPoint(new Translation2d(2.05, 2.75), Rotation2d.fromDegrees(180),
                        Rotation2d.fromDegrees(180));
                break;
            case RightConeMiddleGrid:
                endPoint = new PathPoint(new Translation2d(2.05, 2.20), Rotation2d.fromDegrees(180),
                        Rotation2d.fromDegrees(180));
                break;
            // ---------
            case LeftConeRightGrid:
                endPoint = new PathPoint(new Translation2d(2.05, 1.6), Rotation2d.fromDegrees(180),
                        Rotation2d.fromDegrees(180));
                break;
            case MiddleCubeRightGrid:
                endPoint = new PathPoint(new Translation2d(2.05, 1.1), Rotation2d.fromDegrees(180),
                        Rotation2d.fromDegrees(180));
                break;
            case RightConeRightGrid:
                endPoint = new PathPoint(new Translation2d(2.05, 0.5), Rotation2d.fromDegrees(180),
                        Rotation2d.fromDegrees(180));
                break;
            default:
                new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(180),
                        drivetrain.getPose().getRotation());
                break;
        }
        PathPlannerTrajectory trajectoryToAprilTag = PathPlanner.generatePath(
                new PathConstraints(2.75, 1.25),
                new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(180),
                        drivetrain.getPose().getRotation()), // position, heading
                endPoint // position, heading

        );
        return new TeleopDriveInterruptor()
                .deadlineWith(new FollowPath(drivetrain, trajectoryToAprilTag, field2d, false));
    }
}
