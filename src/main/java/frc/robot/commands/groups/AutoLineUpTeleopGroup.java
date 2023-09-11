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
        private static final double blueArrayX = 1.80; // used to be 2.05
        private static final double blueArrayHeading = 0;// used to be 180

        public static Command get(Drivetrain drivetrain, Field2d field2d) {
                // return new Pose2d(new Translation2d(10, 10), new Rotation2d(0));\
                PathPoint endPoint = new PathPoint(new Translation2d(blueArrayX, 2.75),
                                Rotation2d.fromDegrees(blueArrayHeading),
                                Rotation2d.fromDegrees(blueArrayHeading));
                PathPlannerTrajectory trajectoryToAprilTag = PathPlanner.generatePath(
                                new PathConstraints(2.75, 1.25),
                                new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(0),
                                                drivetrain.getPose().getRotation()), // position, heading
                                endPoint // position,
                                         // heading

                );

                return new TeleopDriveInterruptor()
                                .deadlineWith(new FollowPath(drivetrain, trajectoryToAprilTag, field2d, false));
        }

        public static Command go(Drivetrain drivetrain, Field2d field2d, DriveLocation location) {
                PathPoint endPoint = new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(180),
                                drivetrain.getPose().getRotation());
                System.out.println("RAN!!");
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
                                endPoint = new PathPoint(new Translation2d(blueArrayX, 5.00),
                                                Rotation2d.fromDegrees(blueArrayHeading),
                                                Rotation2d.fromDegrees(blueArrayHeading));
                                break;
                        case MiddleCubeLeftGrid:
                                endPoint = new PathPoint(new Translation2d(blueArrayX, 4.45),
                                                Rotation2d.fromDegrees(blueArrayHeading),
                                                Rotation2d.fromDegrees(blueArrayHeading));
                                break;
                        case RightConeLeftGrid:
                                endPoint = new PathPoint(new Translation2d(blueArrayX, 3.90),
                                                Rotation2d.fromDegrees(blueArrayHeading),
                                                Rotation2d.fromDegrees(blueArrayHeading));
                                break;
                        // ---------
                        case LeftConeMiddleGrid:
                                endPoint = new PathPoint(new Translation2d(blueArrayX, 3.30),
                                                Rotation2d.fromDegrees(blueArrayHeading),
                                                Rotation2d.fromDegrees(blueArrayHeading));
                                break;
                        case MiddleCubeMiddleGrid:
                                System.out.println("Set END POINT");
                                endPoint = new PathPoint(new Translation2d(blueArrayX, 2.75),
                                                Rotation2d.fromDegrees(blueArrayHeading),
                                                Rotation2d.fromDegrees(blueArrayHeading));
                                break;
                        case RightConeMiddleGrid:
                                endPoint = new PathPoint(new Translation2d(blueArrayX, 2.20),
                                                Rotation2d.fromDegrees(blueArrayHeading),
                                                Rotation2d.fromDegrees(blueArrayHeading));
                                break;
                        // ---------
                        case LeftConeRightGrid:
                                endPoint = new PathPoint(new Translation2d(blueArrayX, 1.6),
                                                Rotation2d.fromDegrees(blueArrayHeading),
                                                Rotation2d.fromDegrees(blueArrayHeading));
                                break;
                        case MiddleCubeRightGrid:
                                endPoint = new PathPoint(new Translation2d(blueArrayX, 1.1),
                                                Rotation2d.fromDegrees(blueArrayHeading),
                                                Rotation2d.fromDegrees(blueArrayHeading));
                                break;
                        case RightConeRightGrid:
                                endPoint = new PathPoint(new Translation2d(blueArrayX, 0.5),
                                                Rotation2d.fromDegrees(blueArrayHeading),
                                                Rotation2d.fromDegrees(blueArrayHeading));
                                break;
                        default:
                                endPoint = new PathPoint(drivetrain.getPose().getTranslation(),
                                                Rotation2d.fromDegrees(blueArrayHeading),
                                                drivetrain.getPose().getRotation());
                                break;
                }
                System.out.println("End point");
                PathPlannerTrajectory trajectoryToAprilTag = PathPlanner.generatePath(
                                new PathConstraints(2.75, 1.25),
                                new PathPoint(drivetrain.getPose().getTranslation(), Rotation2d.fromDegrees(0),
                                                drivetrain.getPose().getRotation()), // position, heading
                                endPoint // position,
                                         // heading

                );

                return new TeleopDriveInterruptor()
                                .deadlineWith(new FollowPath(drivetrain, trajectoryToAprilTag, field2d, false));
        }
}
