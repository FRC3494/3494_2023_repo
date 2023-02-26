package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.swervedrivespecialties.swervelib.ThriftyBotModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.arm.ForearmState;

public final class Constants extends AutoConfigurable {
    public static final class Subsystems {
        public static final class Arm {
            public static int TOP_PISTON_SOLENOID_CHANNEL = 2; 
            public static int BOTTOM_PISTON_SOLENOID_CHANNEL = 0; 

            public static int HOPPER_SOLENOID_CHANNEL = 6; 

            public static int FOREARM_MOTOR_CHANNEL = 9; 
            public static double FOREARM_MOTOR_REDUCTION = (1.0 / 60.0) * (10.0 / 22.0);

            public static int FOREARM_ENCODER_CHANNEL = 2;
            public static double FOREARM_ENCODER_OFFSET = 331.12251217887376;

            public static double FOREARM_TARGET_POSITION_TOLERANCE = 10; //degrees

            public static class PIDF { // TODO: TUNE
                public static double P = 0.1;
                public static double I = 0;
                public static double D = 0;
                public static double F = 0;
            }

            public static HashMap<ForearmState, Double> FOREARM_POSITION = new HashMap<>() {{ // TODO: UPDATE THESE TO ACTUAL ENCODER POSITIONS
                put(ForearmState.Base4Cube2, -82.0); // real -80
                put(ForearmState.Base4Cone2, -99.1); // real -99.0
                // Base4Cone1 -76.1 // real -74.8
                put(ForearmState.Base4Cube1, -57.7); // real -56.8
                put(ForearmState.Base1HopperGrab, -34.0); // -33.09
                put(ForearmState.Intermediate, 66.7); // real 47.9
                put(ForearmState.Base2Store, -20.0); // real -20.0
                put(ForearmState.Base1Ground, 15.0); // real 0.0
                put(ForearmState.Base4DoubleSub, -99.1); // real -99.0
                put(ForearmState.Base1Cube1, 101.3); // real 67.4 // change to base2!!!
                put(ForearmState.Base2Cone1, 108.7); // real 90
                put(ForearmState.Base1Hybrid, 26.5); // real 13.0
            }};
        }
        
        public static final class Drivetrain {
            public static final class FrontLeftModule {
                public static int DRIVE_MOTOR_PORT = 17;//12;
                public static int STEER_MOTOR_PORT = 19;//11;

                public static int ENCODER_MOTOR_PORT = 4;

                public static double STEER_OFFSET = -Math.toRadians(290f-36f+0.6-94.4);
            }

            public static final class FrontRightModule {
                public static int DRIVE_MOTOR_PORT = 16;//7;
                public static int STEER_MOTOR_PORT = 18;//8;

                public static int ENCODER_MOTOR_PORT = 5;

                public static double STEER_OFFSET = -Math.toRadians(243f-123.0+2.5-6);
            }

            public static final class BackLeftModule {
                public static int DRIVE_MOTOR_PORT = 2;//14;
                public static int STEER_MOTOR_PORT = 30;//15;

                public static int ENCODER_MOTOR_PORT = 6;

                public static double STEER_OFFSET = -Math.toRadians(290f-36f-0.8-154.7);
            }

            public static final class BackRightModule {
                public static int DRIVE_MOTOR_PORT = 1;//5;
                public static int STEER_MOTOR_PORT = 3;//4;
                public static int ENCODER_MOTOR_PORT = 7;
                public static double STEER_OFFSET = -Math.toRadians(164f+8f+2f+10f+5f+3.8f-8.3-119);
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
			        ThriftyBotModuleConfigurations.STANDARD.getWheelDiameter() * Math.PI*0.1;

            public static final double MAX_VOLTAGE = 12.0;

            public static final double MAX_STANDARD_DEVIATION_LIMELIGHT = 1;
        }
    
        public static final class Claw {
            public static int CLAW_SOLENOID_CHANNEL = 4;
        }

        public static final class Pneumatics {
            public static int BASE_PCM = 21;
            
            public static double MIN_PRESSURE = 100;
            public static double MAX_PRESSURE = 120;
        }
    }

    public static final class Commands {
        public static final class FollowPath {
            public static final PIDController X_CONTROLLER = new PIDController(1, 0, 0);
            public static final PIDController Y_CONTROLLER = new PIDController(1, 0, 0);

            public static final PIDController THETA_CONTROLLER = new PIDController(0.22, 0, 0.05);
            
            /*public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(0.22, 0, 0.05, 
                    new TrapezoidProfile.Constraints(Math.PI, Math.PI));*/
        }

        public static final class AutoBalance {
            public static final double TRIGGER_ANGLE = 10;

            public static final double LEVEL_ANGLE = 2;

            public static final double DRIVE_POWER = 1;

            public static final double DIVIDE_FACTOR = 4;

            public static final double EXIT_TIME = 3;
        }
        public static final class AutoLineUp {
            public static final double MAX_STRAFE_SPEED = 0.3;
            public static final double YAW_ERROR_ALLOWANCE = 1;
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
        public static final int SECONDARY_LEFT_CONTROLLER_PORT = 1;
        public static final int SECONDARY_RIGHT_CONTROLLER_PORT = 2;

        public static final double MAX_DRIVE_SPEED = 9; // m/s
        public static final double MAX_TURN_SPEED = 9; // rad/s
    }
}
