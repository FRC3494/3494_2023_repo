package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.swervedrivespecialties.swervelib.ThriftyBotModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public final class Constants {
    public static final class Subsystems {
        public static final class Drivetrain {
            public static final class FrontLeftModule {
                public static int DRIVE_MOTOR_PORT = 12;
                public static int STEER_MOTOR_PORT = 11;
            }

            public static final class FrontRightModule {
                public static int DRIVE_MOTOR_PORT = 7;
                public static int STEER_MOTOR_PORT = 8;
            }

            public static final class BackLeftModule {
                public static int DRIVE_MOTOR_PORT = 14;
                public static int STEER_MOTOR_PORT = 15;
            }

            public static final class BackRightModule {
                public static int DRIVE_MOTOR_PORT = 5;
                public static int STEER_MOTOR_PORT = 4;
            }

            public static final double TRACKWIDTH_METERS = 0.7112;
            public static final double TRACKLENGTH_METERS = 0.7112;

            public static SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                    // Front left
                    new Translation2d(TRACKWIDTH_METERS / 2.0, TRACKLENGTH_METERS / 2.0),
                    // Front right
                    new Translation2d(TRACKWIDTH_METERS / 2.0, -TRACKLENGTH_METERS / 2.0),
                    // Back left
                    new Translation2d(-TRACKWIDTH_METERS / 2.0, TRACKLENGTH_METERS / 2.0),
                    // Back right
                    new Translation2d(-TRACKWIDTH_METERS / 2.0, -TRACKLENGTH_METERS / 2.0));

            public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
			        ThriftyBotModuleConfigurations.STANDARD.getDriveReduction() *
			        ThriftyBotModuleConfigurations.STANDARD.getWheelDiameter() * Math.PI;

            public static final double MAX_VOLTAGE = 12.0;
        }
    }

    public static final class Commands {
        public static final class FollowPath {
            public static final PIDController X_CONTROLLER = new PIDController(1, 0, 0);
            public static final PIDController Y_CONTROLLER = new PIDController(1, 0, 0);
            
            public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(0.25, 0, 0.01, 
                    new TrapezoidProfile.Constraints(Math.PI, Math.PI));
        }
    }

    public static final class RobotContainer {
        public static final class PathPlanner {
            public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2, 2);

            public static final HashMap<String, Command> PATH_EVENTS = new HashMap<>() {{
                put("print", new PrintCommand("Passed print marker"));
            }};
        }
    }

    public static final class OI {
        public static final int PRIMARY_CONTROLLER_PORT = 0;
    }
}
