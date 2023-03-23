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
import frc.robot.subsystems.forearm.ForearmState;
import frc.robot.subsystems.hopper.HopperState;
import frc.robot.subsystems.shoulder.ShoulderState;

public final class Constants extends AutoConfigurable {
    public static final class Subsystems {
        public static final class Arm {
            public static ArmState INITIAL_STATE = new ArmState(ShoulderState.Base2, ForearmState.Store, HopperState.Retracted);

            public static HashMap<ArmPosition, ArmState> KEY_POSITIONS = new HashMap<>() {
                {
                    put(ArmPosition.LowerHopperGrab, new ArmState(ShoulderState.Base1, ForearmState.LowerHopperGrab, HopperState.Retracted));
                    put(ArmPosition.UpperHopperGrab, new ArmState(ShoulderState.Base1, ForearmState.UpperHopperGrab, HopperState.Retracted));
                    put(ArmPosition.GroundIntake, new ArmState(ShoulderState.Base1, ForearmState.GroundIntake, HopperState.Retracted));
                    put(ArmPosition.DoubleSubstation, new ArmState(ShoulderState.Base4, ForearmState.DoubleSubstation, HopperState.Extended));
                    put(ArmPosition.Hybrid, new ArmState(ShoulderState.Base1, ForearmState.Base1Hybrid, HopperState.Retracted));
                    put(ArmPosition.Store, new ArmState(ShoulderState.Base2, ForearmState.Store, HopperState.Retracted));
                    put(ArmPosition.Base4Cone2, new ArmState(ShoulderState.Base4, ForearmState.Base4Cone2, HopperState.Extended));
                    put(ArmPosition.Base4Cube2, new ArmState(ShoulderState.Base4, ForearmState.Base4Cube2, HopperState.Extended));
                    put(ArmPosition.Base4Cube1, new ArmState(ShoulderState.Base4, ForearmState.Base4Cube1, HopperState.Extended));
                    put(ArmPosition.Base2Cone1, new ArmState(ShoulderState.Base2, ForearmState.Base2Cone1, HopperState.Retracted));
                    put(ArmPosition.Base1Cube1, new ArmState(ShoulderState.Base1, ForearmState.Base1Cube1, HopperState.Retracted));
                }
            };
        }

        public static final class Shoulder {
            public static int TOP_PISTON_SOLENOID_CHANNEL = 2;
            public static int BOTTOM_PISTON_SOLENOID_CHANNEL = 0;

            public static int SHOULDER_POTENTIOMETER_CHANNEL = 4;

            public static double SHOULDER_TARGET_TOLERANCE = 0.012;

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
            public static int FOREARM_MOTOR_CHANNEL = 9;
            public static double FOREARM_MOTOR_REDUCTION = (1.0 / 60.0) * (10.0 / 22.0);

            public static int FOREARM_ENCODER_CHANNEL = 5;
            public static double FOREARM_ENCODER_OFFSET = 331.12251217887376;

            public static double FOREARM_TARGET_POSITION_TOLERANCE = 2; // degrees

            public static class PIDF {
                public static double P = 0.1;
                public static double I = 0;
                public static double D = 0;
                public static double F = 0;
            }

            public static HashMap<ForearmState, Double> POSITIONS = new HashMap<>() {
                { // TODO: UPDATE THESE TO ACTUAL ENCODER POSITIONS
                    put(ForearmState.Base4Cube2, -82.0); // real -80
                    put(ForearmState.Base4Cone2, -105.1); // real -99.0
                    // Base4Cone1 -76.1 // real -74.8
                    put(ForearmState.Base4Cube1, -65.0); // real -56.8
                    put(ForearmState.LowerHopperGrab, -39.0); // -33.09
                    put(ForearmState.UpperHopperGrab, -49.0); // -33.09
                    put(ForearmState.Intermediate, 66.7); // real 47.9
                    put(ForearmState.Store, -8.0); // real -20.0
                    put(ForearmState.GroundIntake, 15.0); // real 0.0
                    put(ForearmState.DoubleSubstation, -99.1); // real -99.0
                    put(ForearmState.Base1Cube1, 101.3); // real 67.4 // change to base2!!!
                    put(ForearmState.Base2Cone1, 108.7); // real 90
                    put(ForearmState.Base1Hybrid, 26.5); // real 13.0
                }
            };
        }
        public static final class Hopper {
            public static int HOPPER_SOLENOID_CHANNEL = 6;
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

            /*public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                    SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
                    SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;*/

            public static final double MAX_VELOCITY_METERS_PER_SECOND = 3.6576f;

            public static final double MAX_VOLTAGE = 12.0;

            public static final double MAX_STANDARD_DEVIATION_LIMELIGHT = 0.01;
        }

        public static final class Claw {
            public static int CLAW_SOLENOID_CHANNEL = 4;
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
            public static final PIDController X_CONTROLLER = new PIDController(1, 0, 0);//.1 "4, 0.1, 0"
            public static final PIDController Y_CONTROLLER = new PIDController(1, 0, 0);//.1
            public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(
                5.0, 0, 0,//2.4,0,0
                new TrapezoidProfile.Constraints(3,
                        3));//max vel, max acc
        
            public static final PIDController PP_THETA_CONTROLLER = new PIDController(0.3, 0.0, 0.0);

            //public static final PIDController X_CONTROLLER = new PIDController(2.5, 1.5, 0);
            //public static final PIDController Y_CONTROLLER = new PIDController(2.5, 1, 0);
            //public static final PIDController THETA_CONTROLLER = new PIDController(2.5, 1.2, 0.5);
            //public static final PIDController THETA_CONTROLLER = new PIDController(0.0, 0.0, 0.05);



            //public static final PIDController X_CONTROLLER = new PIDController(10, 0, 0);
            //public static final PIDController Y_CONTROLLER = new PIDController(10, 0, 0);
//
            //public static final PIDController THETA_CONTROLLER = new PIDController(10, 0, 0);

            /*
             * public static final ProfiledPIDController THETA_CONTROLLER = new
             * ProfiledPIDController(0.22, 0, 0.05,
             * new TrapezoidProfile.Constraints(Math.PI, Math.PI));
             */
        }

        public static final class AutoBalance {
            public static final double TRIGGER_ANGLE = 23;

            public static final double PEAK_ANGLE = 19;//23

            public static final double LEVEL_ANGLE = 5;

            public static final double FAST_POWER = 0.1;
            public static final double SLOW_POWER = 0.0585;//0.05
            public static final double DIVIDE_FACTOR = 3;
            public static final double DIVIDE_ANGLE = 8.5;

            public static final double EXIT_TIME = 3;
        }

        public static final class AutoLineUp {
            public static final double MAX_STRAFE_SPEED = 0.3;
            public static final double YAW_ERROR_ALLOWANCE = 1;
        }
    }

    public static final class RobotContainer {
        public static final class PathPlanner {
            public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(4, 4);

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

        public static final double MAX_DRIVE_SPEED = 3; // m/s
        public static final double MAX_TURN_SPEED = .8; // rad/s
        public static final double DPAD_SPEED = 0.1;

        public static final double FOREARM_FINE_ADJUST_SPEED = 0.8;
    }
}
