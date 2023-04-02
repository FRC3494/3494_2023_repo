package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.claw.ClawState;
import frc.robot.subsystems.forearm.ForearmState;
import frc.robot.subsystems.shoulder.ShoulderState;
import frc.robot.subsystems.wrist.WristState;

public final class Constants extends AutoConfigurable {
    public static final class Subsystems {
        public static final class Arm {
            public static ArmState INITIAL_STATE = new ArmState(ShoulderState.Base2, ForearmState.Store,
                    WristState.Store);

            public static HashMap<ArmPosition, ArmState> KEY_POSITIONS = new HashMap<>() {
                {
                    put(ArmPosition.Store, new ArmState(ShoulderState.Base2, ForearmState.Store, WristState.Store));
                    put(ArmPosition.GroundIntakeCone,
                            new ArmState(ShoulderState.Base1, ForearmState.GroundIntakeCone, WristState.GroundCone));
                    put(ArmPosition.GroundIntakeCube,
                            new ArmState(ShoulderState.Base1, ForearmState.GroundIntakeCube, WristState.GroundCube));
                    put(ArmPosition.DoubleSubstationCone,
                            new ArmState(ShoulderState.Base2, ForearmState.DoubleSubCone, WristState.DoubleSubCone));
                    put(ArmPosition.DoubleSubstationCube,
                            new ArmState(ShoulderState.Base2, ForearmState.DoubleSubCube, WristState.DoubleSubCube));
                    put(ArmPosition.SingleSubstation,
                            new ArmState(ShoulderState.Base2, ForearmState.SingleSubstation, WristState.SingleSub));
                    put(ArmPosition.Base4Cone2,
                            new ArmState(ShoulderState.Base4, ForearmState.Base4Cone2, WristState.Base4Cone2));
                    put(ArmPosition.Base4Cube2,
                            new ArmState(ShoulderState.Base4, ForearmState.Base4Cube2, WristState.Base4Cube2));
                    put(ArmPosition.Base4Cone1,
                            new ArmState(ShoulderState.Base4, ForearmState.Base4Cone1, WristState.Base4Cone1));
                    put(ArmPosition.Base4Cube1,
                            new ArmState(ShoulderState.Base4, ForearmState.Base4Cube1, WristState.Base4Cube1));
                    put(ArmPosition.Base2Cone1,
                            new ArmState(ShoulderState.Base2, ForearmState.Base2Cone1, WristState.Base2Cone1));
                    put(ArmPosition.Base2Cube1,
                            new ArmState(ShoulderState.Base2, ForearmState.Base2Cube1, WristState.Base2Cube1));
                    put(ArmPosition.Base1Hybrid,
                            new ArmState(ShoulderState.Base1, ForearmState.Base1Hybrid, WristState.Hybrid));
                }
            };
        }

        public static final class Shoulder {
            public static int TOP_PISTON_SOLENOID_CHANNEL = 2;
            public static int BOTTOM_PISTON_SOLENOID_CHANNEL = 0;

            public static int POTENTIOMETER_CHANNEL = 4;

            public static double TARGET_TOLERANCE = 0.012;

            public static HashMap<ShoulderState, Double> POSITIONS = new HashMap<>() {
                {
                    put(ShoulderState.Base1, 0.191);
                    put(ShoulderState.Base2, 0.267);
                    put(ShoulderState.Base3, 0.328);
                    put(ShoulderState.Base4, 0.390);
                }
            };
        }

        public static final class Forearm {
            public static int MOTOR_CHANNEL = 5;
            public static double MOTOR_REDUCTION = (1.0 / 5.0) * (1.0 / 4.0) * (1.0 / 3.0) * (10.0 / 40.0);

            public static double MAX_SPEED = 0.8;
            public static double MIN_SPEED = -0.8;

            public static double SLOW_MAX_SPEED = 0.15;
            public static double SLOW_MIN_SPEED = -0.15;

            public static float MIN_POSITION = -86.241f; // 135.5
            public static float MAX_POSITION = 89.505f; // -136.0

            public static double TARGET_POSITION_TOLERANCE = 2; // degrees

            public static double RAMP_RATE = 0.5;

            public static int CURRENT_LIMIT = 40; // 50

            public static class PIDF {
                public static double P = 0.05;
                public static double I = 0;
                public static double D = 0.075;
                public static double F = 0;
            }

            public static HashMap<ForearmState, Double> POSITIONS = new HashMap<>() {
                {
                    put(ForearmState.Intermediate, 42.8);
                    put(ForearmState.GroundIntakeCube, 62.9);
                    put(ForearmState.GroundIntakeCone, 79.52);
                    put(ForearmState.DoubleSubCube, -72.00);
                    put(ForearmState.DoubleSubCone, -86.10);
                    put(ForearmState.SingleSubstation, 28.09);
                    put(ForearmState.Base1Hybrid, 37.8);
                    put(ForearmState.Store, 11.9);
                    put(ForearmState.Base4Cone2, -134.62);
                    put(ForearmState.Base4Cube2, -120.08);
                    put(ForearmState.Base4Cone1, -117.2);
                    put(ForearmState.Base4Cube1, -3.1);
                    put(ForearmState.Base2Cone1, 129.0);
                    put(ForearmState.Base2Cube1, 100.9);

                    put(ForearmState.AUTO_Base2Cube1, -46.3);
                }
            };
        }

        public static final class Wrist {
            public static int MOTOR_CHANNEL = 9;
            public static double MOTOR_REDUCTION = (13.0 / 68.0) * (13.0 / 68.0) * (20.0 / 60.0);
            // public static double MOTOR_REDUCTION = (1.0 / 5.0) * (1.0 / 5.0) * (18.0 /
            // 60.0);

            public static double MAX_SPEED = 0.4;
            public static double MIN_SPEED = -0.4;

            public static float MIN_POSITION = -33.403f + (float) frc.robot.subsystems.wrist.Wrist.degrees2Motor(90);
            public static float MAX_POSITION = 37.905f + (float) frc.robot.subsystems.wrist.Wrist.degrees2Motor(90);

            public static double TARGET_POSITION_TOLERANCE = 2; // degrees

            public static double RAMP_RATE = 0.5;

            public static int CURRENT_LIMIT = 40;

            public static double MAX_CORRECT_VELOCITY = Math.toRadians(0.5);
            public static double CORRECT_PERIOD = 0.5;

            public static class PIDF {
                public static double P = 0.1;
                public static double I = 0;
                public static double D = 0;
                public static double F = 0;
            }

            public static HashMap<WristState, Double> POSITIONS = new HashMap<>() {
                {
                    put(WristState.Store, -46.0);
                    put(WristState.GroundCube, 139.9);
                    put(WristState.GroundCone, 109.7);
                    put(WristState.Base2Cone1, 76.9);
                    put(WristState.Base4Cone1, 233.2);
                    put(WristState.Base4Cone2, 150.0);
                    put(WristState.Base4Cube1, -25.8);
                    put(WristState.Base4Cube2, 162.8);
                    put(WristState.DoubleSubCube, 146.1);
                    put(WristState.DoubleSubCone, 168.3);
                    put(WristState.SingleSub, 229.7);
                    put(WristState.Hybrid, 185.2);
                    put(WristState.Base2Cube1, 110.4);

                    put(WristState.AUTO_Base2Cube1, 107.2);
                }
            };
        }

        public static final class Claw {
            public static int MOTOR_CHANNEL = 8;

            public static HashMap<ClawState, Double> SPEEDS = new HashMap<>() {
                {
                    put(ClawState.Idle, 0.125);
                    put(ClawState.IntakeCone, 0.75);// 0.5
                    put(ClawState.IntakeCube, 0.75);
                    put(ClawState.FullIntake, 1.0);
                    put(ClawState.OuttakeCone, -0.75);// -0.5
                    put(ClawState.OuttakeCube, -0.5);
                    put(ClawState.FullOuttake, -1.0);// -0.5
                }
            };

            public static int CURRENT_LIMIT = 40;
            public static double CURRENT_CUTOFF = 38.0;
            public static double CURRENT_CUTOFF_DURATION = 0.25;
        }

        public static final class Drivetrain {
            public static final class FrontLeftModule {
                public static int DRIVE_MOTOR_PORT = 19;// 12;
                public static int STEER_MOTOR_PORT = 17;// 11;

                public static int ENCODER_MOTOR_PORT = 1;

                public static double STEER_OFFSET = Math.toRadians(17.783);
            }

            public static final class FrontRightModule {
                public static int DRIVE_MOTOR_PORT = 16;// 7;
                public static int STEER_MOTOR_PORT = 18;// 8;

                public static int ENCODER_MOTOR_PORT = 2;

                public static double STEER_OFFSET = Math.toRadians(16.056);
            }

            public static final class BackLeftModule {
                public static int DRIVE_MOTOR_PORT = 30;// 14;
                public static int STEER_MOTOR_PORT = 2;// 15;

                public static int ENCODER_MOTOR_PORT = 0;

                public static double STEER_OFFSET = Math.toRadians(241.404);
            }

            public static final class BackRightModule {
                public static int DRIVE_MOTOR_PORT = 3;// 5;
                public static int STEER_MOTOR_PORT = 1;// 4;

                public static int ENCODER_MOTOR_PORT = 3;

                public static double STEER_OFFSET = -Math.toRadians(8.247);
            }

            public static final double TRACKWIDTH_METERS = 0.47625;
            public static final double TRACKLENGTH_METERS = 0.52705;

            public static SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                    // Front left
                    new Translation2d(TRACKLENGTH_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
                    // Front right
                    new Translation2d(TRACKLENGTH_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),
                    // Back left
                    new Translation2d(-TRACKLENGTH_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
                    // Back right
                    new Translation2d(-TRACKLENGTH_METERS / 2.0, -TRACKWIDTH_METERS / 2.0));

            /*
             * public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
             * SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
             * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
             */

            public static final double MAX_VELOCITY_METERS_PER_SECOND = 3.6576f;

            public static final double MAX_VOLTAGE = 12.0;

            public static final double MAX_STANDARD_DEVIATION_LIMELIGHT = 0.01;
        }

        public static final class Pneumatics {
            public static int BASE_PCM = 21;

            public static double MIN_PRESSURE = 100;
            public static double MAX_PRESSURE = 110;
        }

        public static final class Leds {
            public static int LED_PORT = 0;

            public static int STRIP_LENGTH = 12;
        }
    }

    public static final class Commands {
        public static final class FollowPath {
            public static final PIDController X_CONTROLLER = new PIDController(1, 0, 0);// .1 "4, 0.1, 0"
            public static final PIDController Y_CONTROLLER = new PIDController(1, 0, 0);// .1
            public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(
                    5.0, 0, 0, // 2.4,0,0
                    new TrapezoidProfile.Constraints(3,
                            3));// max vel, max acc

            public static final PIDController PP_THETA_CONTROLLER = new PIDController(0.3, 0.0, 0.0);

            // public static final PIDController X_CONTROLLER = new PIDController(2.5, 1.5,
            // 0);
            // public static final PIDController Y_CONTROLLER = new PIDController(2.5, 1,
            // 0);
            // public static final PIDController THETA_CONTROLLER = new PIDController(2.5,
            // 1.2, 0.5);
            // public static final PIDController THETA_CONTROLLER = new PIDController(0.0,
            // 0.0, 0.05);

            // public static final PIDController X_CONTROLLER = new PIDController(10, 0, 0);
            // public static final PIDController Y_CONTROLLER = new PIDController(10, 0, 0);
            //
            // public static final PIDController THETA_CONTROLLER = new PIDController(10, 0,
            // 0);

            /*
             * public static final ProfiledPIDController THETA_CONTROLLER = new
             * ProfiledPIDController(0.22, 0, 0.05,
             * new TrapezoidProfile.Constraints(Math.PI, Math.PI));
             */
        }

        public static final class AutoBalance {
            public static final double TRIGGER_ANGLE = 23;

            public static final double PEAK_ANGLE = 14;// 19

            public static final double SUB_LEVEL_ANGLE = 13;// 19

            public static final double LEVEL_ANGLE = 5;

            public static final double FAST_POWER = 0.75;
            public static final double SLOW_POWER = 0.585;// 0.05
            public static final double DIVIDE_FACTOR = 3;
            public static final double DIVIDE_ANGLE = 8.5;

            public static final double EXIT_TIME = 3;
        }

        public static final class AutoExtendBalance {
            public static final double TRIGGER_ANGLE = 22;

            public static final double SLOW_POWER = 0.75;
        }

        public static final class AutoLineUp {
            public static final double MAX_STRAFE_SPEED = 0.3;
            public static final double YAW_ERROR_ALLOWANCE = 1;
        }
    }

    public static final class RobotContainer {
        public static final class PathPlanner {
            public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2, 1);

            public static HashMap<String, Command> PATH_EVENTS = new HashMap<>() {
                {

                }
            };
        }
    }

    public static final class OI {
        public static final int PRIMARY_CONTROLLER_PORT = 0;
        public static final int SECONDARY_LEFT_CONTROLLER_PORT = 1;
        public static final int SECONDARY_RIGHT_CONTROLLER_PORT = 2;

        public static final double DRIVE_SPEED = 3; // m/s
        public static final double TURN_SPEED = 5.5; // rad/s

        public static final double SLOW_DRIVE_SPEED = 1.5; // m/s
        public static final double SLOW_TURN_SPEED = 2.75; // rad/s

        public static final double DPAD_SPEED = 0.1;

        public static final double FOREARM_FINE_ADJUST_SPEED = 0.15;
        public static final double WRIST_FINE_ADJUST_SPEED = 0.15;
    }
}
