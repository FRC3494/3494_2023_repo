package frc.robot;

import java.util.function.Function;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RunPneumatics;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoLockDrivetrain;
import frc.robot.commands.auto.AutoSetArm;
import frc.robot.commands.auto.AutoSetClaw;
import frc.robot.commands.auto.AutoSetForearm;
import frc.robot.commands.auto.AutoSetShoulder;
import frc.robot.commands.auto.AutoSetWrist;
import frc.robot.commands.auto.AutoWaitForGrab;
import frc.robot.commands.auto.FollowPath;
import frc.robot.commands.groups.AutoBalanceGroup;
import frc.robot.commands.groups.AutoBalanceGroupDumb;
import frc.robot.commands.groups.AutoBalanceGroupDumbReverse;
import frc.robot.commands.groups.AutoBalanceTeleopGroup;
import frc.robot.commands.teleop.TeleopDrive;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawState;
import frc.robot.subsystems.forearm.Forearm;
import frc.robot.subsystems.forearm.ForearmState;
import frc.robot.subsystems.leds.LedPattern;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderState;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristState;

public class RobotContainer {
        public final Drivetrain drivetrain;
        public final Pneumatics pneumatics;
        public final Shoulder shoulder;
        public final Forearm forearm;
        public final Wrist wrist;
        public final Arm arm;
        public final Claw claw;
        public final Leds leds;
        public final Camera camera;
        private ShuffleboardTab mainTab;
        private ShuffleboardTab fieldTab;
        private ShuffleboardTab armTab;

        private SendableChooser<String> autoChooser;

        private Field2d robotPosition;

        private Command autoBalanceDrivetrainCommand;
        private boolean alternateAutoBalance = true;

        public RobotContainer() {
                PathPlannerServer.startServer(3494);

                NavX.getNavX();

                drivetrain = new Drivetrain();

                pneumatics = new Pneumatics();

                shoulder = new Shoulder();
                forearm = new Forearm();
                wrist = new Wrist();
                arm = new Arm(shoulder, forearm, wrist);

                claw = new Claw();

                camera = new Camera();

                leds = new Leds();

                autoBalanceDrivetrainCommand = AutoBalanceTeleopGroup.get(drivetrain);

                // autoBalanceDrivetrainCommand = AutoBalanceGroup.get(drivetrain);

                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                // drivetrain.setDefaultCommand(new AutoLineUp(drivetrain));
                drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, forearm, wrist));
                // drivetrain.setDefaultCommand(new driveForward(drivetrain));
                pneumatics.setDefaultCommand(new RunPneumatics(pneumatics));

                initShuffleboardObjects();

                Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("print", new PrintCommand("Passed print marker"));
                Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("Balance", new AutoBalance(drivetrain));
                Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("ArmCone2",
                                new AutoSetArm(arm, ArmPosition.Base4Cone2));
                Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("ArmStore",
                                new AutoSetArm(arm, ArmPosition.Store));
                Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("ClawOpen",
                                new AutoSetClaw(claw, ClawState.IntakeCube));
                Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("ClawClosed",
                                new AutoSetClaw(claw, ClawState.OuttakeCube));
                Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("Wait5", new WaitCommand(5));
        }

        public boolean errorRecording = false;
        public double errorStart = 0;

        public static Command pathFollow(RobotContainer container, String pathName) {
                return pathFollow(container, pathName, Constants.RobotContainer.PathPlanner.PATH_CONSTRAINTS);
        }

        public static Command pathFollow(RobotContainer container, String pathName, double targetSpeed) {
                return pathFollow(container, pathName, new PathConstraints(targetSpeed,
                                Constants.RobotContainer.PathPlanner.PATH_CONSTRAINTS.maxAcceleration));
        }

        public static Command pathFollow(RobotContainer container, String pathName, PathConstraints constraints) {
                PathPlannerTrajectory loadedPath = PathPlanner.loadPath(pathName, constraints);

                return Commands.either(
                                new FollowPath(container.drivetrain, loadedPath, container.robotPosition, true,
                                                NavX.getYaw() - container.errorStart),
                                new FollowPath(container.drivetrain, loadedPath, container.robotPosition, true),
                                () -> container.errorRecording).andThen(() -> {
                                        container.errorRecording = false;
                                }, container.drivetrain);
        }

        public static Command startRecordingError(RobotContainer container) {
                return Commands.runOnce(() -> {
                        container.errorRecording = true;
                        container.errorStart = NavX.getYaw();
                }, container.drivetrain);
        }

        public enum Autos {
                PlaceThenMobilityThenBalance("Place Medium Then Mobility Then Balance", (container) -> {
                        return new SequentialCommandGroup(
                                        new AutoSetArm(container.arm, ArmPosition.Base2Cone1),
                                        new AutoSetWrist(container.wrist, WristState.Base2Cone1),
                                        new WaitCommand(1),
                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                        new WaitCommand(0.2),
                                        new AutoSetWrist(container.wrist, WristState.Store),
                                        new AutoSetArm(container.arm, ArmPosition.Store),
                                        new AutoSetClaw(container.claw, ClawState.Idle),
                                        pathFollow(container, "Auto1 Segment1", 1.85),
                                        AutoBalanceGroup.get(container.drivetrain));
                }),

                PlaceThenMobilityThenDumbBalance("Place Medium Then Mobility Then Dumb Balance", (container) -> {
                        return new SequentialCommandGroup(
                                        new AutoSetArm(container.arm, ArmPosition.Base2Cone1),
                                        new AutoSetWrist(container.wrist, WristState.Base2Cone1),
                                        new WaitCommand(1.75),
                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                        new WaitCommand(0.2),
                                        new AutoSetWrist(container.wrist, WristState.Store),
                                        new AutoSetArm(container.arm, ArmPosition.Store),
                                        new AutoSetClaw(container.claw, ClawState.Idle),
                                        pathFollow(container, "Auto1 Segment1", 1.85),
                                        AutoBalanceGroupDumb.get(container.drivetrain),
                                        pathFollow(container, "Dumb Balance", 0.5));
                }),
                PlaceThenMobility("Place Medium Then Mobility", (container) -> {
                        return new SequentialCommandGroup(
                                        new AutoSetArm(container.arm, ArmPosition.Base2Cone1),
                                        new AutoSetWrist(container.wrist, WristState.Base2Cone1),
                                        new WaitCommand(1),
                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                        new WaitCommand(0.2),
                                        new AutoSetWrist(container.wrist, WristState.Store),
                                        new AutoSetArm(container.arm, ArmPosition.Store),
                                        new AutoSetClaw(container.claw, ClawState.Idle),
                                        pathFollow(container, "Auto1 Segment1", 1.85));
                }),
                PlaceThenBalance("Place Medium Then Balance", (container) -> {
                        return new SequentialCommandGroup(
                                        new AutoSetArm(container.arm, ArmPosition.Base2Cone1),
                                        new AutoSetWrist(container.wrist, WristState.Base2Cone1),
                                        new WaitCommand(1),
                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                        new WaitCommand(0.2),
                                        new AutoSetWrist(container.wrist, WristState.Store),
                                        new AutoSetArm(container.arm, ArmPosition.Store),
                                        new AutoSetClaw(container.claw, ClawState.Idle),
                                        AutoBalanceGroup.get(container.drivetrain));
                }),
                PlaceHighMobilityBalance("Place High Mobiliy Balance", (container) -> {
                        return new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                        new AutoSetForearm(container.forearm, ForearmState.Base4Cone2),
                                                        new AutoSetWrist(container.wrist, WristState.Base4Cone2)),
                                        new WaitCommand(0.5),
                                        new AutoSetShoulder(container.shoulder, ShoulderState.Base4),
                                        new WaitCommand(2),
                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                        new WaitCommand(0.2),
                                        new ParallelCommandGroup(
                                                        pathFollow(container, "SmallBackup", 1.85),
                                                        new SequentialCommandGroup(
                                                                        new WaitCommand(0.75),
                                                                        new ParallelCommandGroup(
                                                                                        new AutoSetShoulder(
                                                                                                        container.shoulder,
                                                                                                        ShoulderState.Base2),
                                                                                        new AutoSetForearm(
                                                                                                        container.forearm,
                                                                                                        ForearmState.Store),
                                                                                        new AutoSetWrist(
                                                                                                        container.wrist,
                                                                                                        WristState.Store)),
                                                                        new AutoSetClaw(container.claw,
                                                                                        ClawState.Idle))),
                                        pathFollow(container, "MobilityNoTurn", 1.85),
                                        AutoBalanceGroupDumb.get(container.drivetrain),
                                        pathFollow(container, "Dumb Balance", 0.5));

                }),
                Place("Place Medium", (container) -> {
                        return new SequentialCommandGroup(
                                        new AutoSetArm(container.arm, ArmPosition.Base2Cone1),
                                        new AutoSetWrist(container.wrist, WristState.Base2Cone1),
                                        new WaitCommand(1),
                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                        new WaitCommand(0.2),
                                        new AutoSetWrist(container.wrist, WristState.Store),
                                        new AutoSetArm(container.arm, ArmPosition.Store),
                                        new AutoSetClaw(container.claw, ClawState.Idle));
                }),

                PlaceHigh("Place High", (container) -> {
                        return new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                        new AutoSetForearm(container.forearm, ForearmState.Base4Cone2),
                                                        new AutoSetWrist(container.wrist, WristState.Base4Cone2)),
                                        new WaitCommand(0.5),
                                        new AutoSetShoulder(container.shoulder, ShoulderState.Base4),
                                        new WaitCommand(2),
                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                        new WaitCommand(0.2),
                                        new ParallelCommandGroup(
                                                        pathFollow(container, "LeaveCom"),
                                                        new SequentialCommandGroup(
                                                                        new WaitCommand(0.75),
                                                                        new ParallelCommandGroup(
                                                                                        new AutoSetShoulder(
                                                                                                        container.shoulder,
                                                                                                        ShoulderState.Base2),
                                                                                        new AutoSetForearm(
                                                                                                        container.forearm,
                                                                                                        ForearmState.Store),
                                                                                        new AutoSetWrist(
                                                                                                        container.wrist,
                                                                                                        WristState.Store)),
                                                                        new AutoSetClaw(container.claw,
                                                                                        ClawState.Idle))));
                }),

                PlaceHighBalanceForward("Place High Dumb Balance Forward", (container) -> {
                        return new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                        new AutoSetForearm(container.forearm, ForearmState.Base4Cone2),
                                                        new AutoSetWrist(container.wrist, WristState.Base4Cone2)),
                                        new WaitCommand(0.5),
                                        new AutoSetShoulder(container.shoulder, ShoulderState.Base4),
                                        new WaitCommand(2),
                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                        new WaitCommand(0.2),
                                        pathFollow(container, "Dumb Balance Reverse Place Nudge", 0.5),
                                        new ParallelCommandGroup(
                                                        new AutoSetShoulder(container.shoulder, ShoulderState.Base2),
                                                        new AutoSetForearm(container.forearm, ForearmState.Store),
                                                        new AutoSetWrist(container.wrist, WristState.Store)),
                                        new ParallelCommandGroup(
                                                        new SequentialCommandGroup(
                                                                        AutoBalanceGroupDumbReverse
                                                                                        .get(container.drivetrain),
                                                                        pathFollow(container, "Dumb Balance Reverse",
                                                                                        0.5)),
                                                        new SequentialCommandGroup(
                                                                        new AutoSetClaw(container.claw,
                                                                                        ClawState.Idle))),
                                        new AutoLockDrivetrain(container.drivetrain));
                }),

                PlaceThenBalanceForward("Place Medium Then Dumb Balance Forward", (container) -> {
                        return new SequentialCommandGroup(
                                        new AutoSetArm(container.arm, ArmPosition.Base2Cone1),
                                        new AutoSetWrist(container.wrist, WristState.Base2Cone1),
                                        new WaitCommand(1),
                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                        new WaitCommand(0.2),
                                        new AutoSetWrist(container.wrist, WristState.Store),
                                        new AutoSetArm(container.arm, ArmPosition.Store),
                                        new AutoSetClaw(container.claw, ClawState.Idle),
                                        new ParallelCommandGroup(
                                                        new SequentialCommandGroup(
                                                                        AutoBalanceGroupDumbReverse
                                                                                        .get(container.drivetrain),
                                                                        pathFollow(container, "Dumb Balance Reverse",
                                                                                        0.5)),
                                                        new SequentialCommandGroup(
                                                                        new AutoSetClaw(container.claw,
                                                                                        ClawState.Idle))),
                                        new AutoLockDrivetrain(container.drivetrain));
                }),

                Balance("Balance", (container) -> {
                        return AutoBalanceGroup.get(container.drivetrain);
                }),

                None("None", (container) -> {
                        return new PrintCommand("l");
                }),

                PlacePickupPlaceLeft("Place Medium then Pickup Cube then Place Medium Left", (container) -> {
                        return new SequentialCommandGroup(
                                        new AutoSetArm(container.arm, ArmPosition.Base2Cone1),
                                        new AutoSetWrist(container.wrist, WristState.Base2Cone1),
                                        new WaitCommand(1),
                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                        new WaitCommand(0.2),
                                        new ParallelCommandGroup(
                                                        new AutoSetArm(container.arm, ArmPosition.GroundIntakeCube),
                                                        new AutoSetClaw(container.claw, ClawState.IntakeCube),
                                                        pathFollow(container, "LeaveComPickUpRight")),
                                        new WaitCommand(0.75),
                                        new AutoSetClaw(container.claw, ClawState.IntakeCube),
                                        new ParallelCommandGroup(
                                                        new AutoSetArm(container.arm, ArmPosition.Base2Cube1),
                                                        pathFollow(container, "LeaveComPickUpReturnRight")),
                                        new WaitCommand(0.2),
                                        new AutoSetClaw(container.claw, ClawState.OuttakeCube),
                                        new WaitCommand(0.3),
                                        new AutoSetArm(container.arm, ArmPosition.Store),
                                        new AutoSetClaw(container.claw, ClawState.Idle));
                }),
                lacePickupPlaceRight("Place Medium then Pickup Cube then Place Medium Right", (container) -> {
                        return new SequentialCommandGroup(
                                        new AutoSetArm(container.arm, ArmPosition.Base2Cone1),
                                        new AutoSetWrist(container.wrist, WristState.Base2Cone1),
                                        new WaitCommand(1),
                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                        new WaitCommand(0.4),
                                        new ParallelCommandGroup(
                                                        new AutoSetArm(container.arm, ArmPosition.GroundIntakeCube),
                                                        new AutoSetClaw(container.claw, ClawState.IntakeCube),
                                                        pathFollow(container, "LeaveComPickUpLeft")),
                                        new WaitCommand(0.75),
                                        new AutoSetClaw(container.claw, ClawState.IntakeCube),
                                        new ParallelCommandGroup(
                                                        new AutoSetArm(container.arm, ArmPosition.Base2Cube1),
                                                        pathFollow(container, "LeaveComPickUpReturnLeft")),
                                        new WaitCommand(0.2),
                                        new AutoSetClaw(container.claw, ClawState.OuttakeCube),
                                        new WaitCommand(0.3),
                                        new AutoSetArm(container.arm, ArmPosition.Store),
                                        new AutoSetClaw(container.claw, ClawState.Idle));
                }),
                PlaceMediumRIGHTPickupCubeBack("Place Medium from back Pickup Cube then place Medium from back RIGHT",
                                (container) -> {
                                        return new SequentialCommandGroup(
                                                        startRecordingError(container),
                                                        new ParallelCommandGroup(
                                                                        new AutoSetForearm(container.forearm,
                                                                                        ForearmState.Base4Cone1),
                                                                        new AutoSetWrist(container.wrist,
                                                                                        WristState.AUTO_Base4Cone1)),
                                                        new WaitCommand(0.5),
                                                        new AutoSetShoulder(container.shoulder, ShoulderState.Base4),
                                                        new WaitCommand(1.0),
                                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                                        new WaitCommand(0.2),

                                                        new ParallelCommandGroup(
                                                                        pathFollow(container,
                                                                                        "LeaveComPickUpLeftNo180"),
                                                                        new SequentialCommandGroup(
                                                                                        new WaitCommand(0.65),
                                                                                        new ParallelCommandGroup(
                                                                                                        new AutoSetForearm(
                                                                                                                        container.forearm,
                                                                                                                        ForearmState.GroundIntakeCube),
                                                                                                        new AutoSetWrist(
                                                                                                                        container.wrist,
                                                                                                                        WristState.AUTO_GroundIntake),
                                                                                                        new AutoSetClaw(container.claw,
                                                                                                                        ClawState.IntakeCube),
                                                                                                        new SequentialCommandGroup(
                                                                                                                        new WaitCommand(1.5),
                                                                                                                        new AutoSetShoulder(
                                                                                                                                        container.shoulder,
                                                                                                                                        ShoulderState.Base1))))),
                                                        new WaitCommand(0.25),
                                                        new ParallelCommandGroup(
                                                                        new AutoSetClaw(container.claw,
                                                                                        ClawState.IntakeCube),
                                                                        new SequentialCommandGroup(
                                                                                        new AutoSetShoulder(
                                                                                                        container.shoulder,
                                                                                                        ShoulderState.Base2),
                                                                                        new ParallelCommandGroup(
                                                                                                        new AutoSetForearm(
                                                                                                                        container.forearm,
                                                                                                                        ForearmState.AUTO_Base2Cube1),
                                                                                                        new AutoSetWrist(
                                                                                                                        container.wrist,
                                                                                                                        WristState.AUTO_Base2Cube1))),
                                                                        pathFollow(container,
                                                                                        "LeaveComPickUpReturnLeftNo180",
                                                                                        new PathConstraints(3, 2))),
                                                        new AutoSetClaw(container.claw,
                                                                        ClawState.FullOuttake),
                                                        new WaitCommand(0.25),
                                                        pathFollow(container, "LeaveComCurveRight",
                                                                        new PathConstraints(3, 2))
                                        // new WaitCommand(0.5),
                                        // new AutoSetClaw(container.claw, ClawState.FullOuttake),
                                        // new WaitCommand(0.5));
                                        );
                                }),
                PlaceMediumLEFTPickupCubeBack("Place Medium from back Pickup Cube then place Medium from back LEFT",
                                (container) -> {
                                        return new SequentialCommandGroup(
                                                        startRecordingError(container),
                                                        new ParallelCommandGroup(
                                                                        new AutoSetForearm(container.forearm,
                                                                                        ForearmState.Base4Cone1),
                                                                        new AutoSetWrist(container.wrist,
                                                                                        WristState.AUTO_Base4Cone1)),
                                                        new WaitCommand(0.5),
                                                        new AutoSetShoulder(container.shoulder, ShoulderState.Base4),
                                                        new WaitCommand(1.0),
                                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                                        new WaitCommand(0.2),

                                                        new ParallelCommandGroup(
                                                                        pathFollow(container,
                                                                                        "LeaveComPickUpRightNo180"),
                                                                        new SequentialCommandGroup(
                                                                                        new WaitCommand(0.65),
                                                                                        new ParallelCommandGroup(
                                                                                                        new AutoSetForearm(
                                                                                                                        container.forearm,
                                                                                                                        ForearmState.GroundIntakeCube),
                                                                                                        new AutoSetWrist(
                                                                                                                        container.wrist,
                                                                                                                        WristState.AUTO_GroundIntake),
                                                                                                        new AutoSetClaw(container.claw,
                                                                                                                        ClawState.IntakeCube),
                                                                                                        new SequentialCommandGroup(
                                                                                                                        new WaitCommand(1.5),
                                                                                                                        new AutoSetShoulder(
                                                                                                                                        container.shoulder,
                                                                                                                                        ShoulderState.Base1))))),
                                                        new WaitCommand(0.25),
                                                        new ParallelCommandGroup(
                                                                        new AutoSetClaw(container.claw,
                                                                                        ClawState.IntakeCube),
                                                                        new SequentialCommandGroup(
                                                                                        new AutoSetShoulder(
                                                                                                        container.shoulder,
                                                                                                        ShoulderState.Base2),
                                                                                        new ParallelCommandGroup(
                                                                                                        new AutoSetForearm(
                                                                                                                        container.forearm,
                                                                                                                        ForearmState.AUTO_Base2Cube1),
                                                                                                        new AutoSetWrist(
                                                                                                                        container.wrist,
                                                                                                                        WristState.AUTO_Base2Cube1))),
                                                                        pathFollow(container,
                                                                                        "LeaveComPickUpReturnRightNo180",
                                                                                        new PathConstraints(3, 2))),
                                                        new AutoSetClaw(container.claw,
                                                                        ClawState.FullOuttake),
                                                        new WaitCommand(0.25),
                                                        pathFollow(container, "LeaveComCurveLeft",
                                                                        new PathConstraints(3, 2))
                                        // new WaitCommand(0.5),
                                        // new AutoSetClaw(container.claw, ClawState.FullOuttake),
                                        // new WaitCommand(0.5));
                                        );
                                }),
                PlaceMediumRIGHTMaybePickupCubeBack(
                                "Place Medium from back Pickup Cube (stop if fail) then place Medium from back RIGHT",
                                (container) -> {
                                        return new SequentialCommandGroup(
                                                        startRecordingError(container),
                                                        new ParallelCommandGroup(
                                                                        new AutoSetForearm(container.forearm,
                                                                                        ForearmState.Base4Cone1),
                                                                        new AutoSetWrist(container.wrist,
                                                                                        WristState.AUTO_Base4Cone1)),
                                                        new WaitCommand(0.5),
                                                        new AutoSetShoulder(container.shoulder, ShoulderState.Base4),
                                                        new WaitCommand(1.0),
                                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                                        new WaitCommand(0.2),

                                                        new ParallelCommandGroup(
                                                                        pathFollow(container,
                                                                                        "LeaveComPickUpLeftNo180"),
                                                                        new SequentialCommandGroup(
                                                                                        new WaitCommand(0.65),
                                                                                        new ParallelCommandGroup(
                                                                                                        new AutoSetForearm(
                                                                                                                        container.forearm,
                                                                                                                        ForearmState.GroundIntakeCube),
                                                                                                        new AutoSetWrist(
                                                                                                                        container.wrist,
                                                                                                                        WristState.AUTO_GroundIntake),
                                                                                                        new AutoSetClaw(container.claw,
                                                                                                                        ClawState.IntakeCube),
                                                                                                        new SequentialCommandGroup(
                                                                                                                        new WaitCommand(1.5),
                                                                                                                        new AutoSetShoulder(
                                                                                                                                        container.shoulder,
                                                                                                                                        ShoulderState.Base1))))),
                                                        new WaitCommand(0.25),
                                                        new AutoWaitForGrab(container.claw),
                                                        new ParallelCommandGroup(
                                                                        new AutoSetClaw(container.claw,
                                                                                        ClawState.IntakeCube),
                                                                        new SequentialCommandGroup(
                                                                                        new AutoSetShoulder(
                                                                                                        container.shoulder,
                                                                                                        ShoulderState.Base2),
                                                                                        new ParallelCommandGroup(
                                                                                                        new AutoSetForearm(
                                                                                                                        container.forearm,
                                                                                                                        ForearmState.AUTO_Base2Cube1),
                                                                                                        new AutoSetWrist(
                                                                                                                        container.wrist,
                                                                                                                        WristState.AUTO_Base2Cube1))),
                                                                        pathFollow(container,
                                                                                        "LeaveComPickUpReturnLeftNo180",
                                                                                        new PathConstraints(3, 2))),
                                                        new AutoSetClaw(container.claw,
                                                                        ClawState.FullOuttake),
                                                        new WaitCommand(0.25),
                                                        pathFollow(container, "LeaveComCurveRight",
                                                                        new PathConstraints(3, 2))
                                        // new WaitCommand(0.5),
                                        // new AutoSetClaw(container.claw, ClawState.FullOuttake),
                                        // new WaitCommand(0.5));
                                        );
                                }),
                PlaceMediumLEFTMaybePickupCubeBack(
                                "Place Medium from back Pickup Cube (stop if fail) then place Medium from back LEFT",
                                (container) -> {
                                        return new SequentialCommandGroup(
                                                        startRecordingError(container),
                                                        new ParallelCommandGroup(
                                                                        new AutoSetForearm(container.forearm,
                                                                                        ForearmState.Base4Cone1),
                                                                        new AutoSetWrist(container.wrist,
                                                                                        WristState.AUTO_Base4Cone1)),
                                                        new WaitCommand(0.5),
                                                        new AutoSetShoulder(container.shoulder, ShoulderState.Base4),
                                                        new WaitCommand(1.0),
                                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                                        new WaitCommand(0.2),

                                                        new ParallelCommandGroup(
                                                                        pathFollow(container,
                                                                                        "LeaveComPickUpRightNo180"),
                                                                        new SequentialCommandGroup(
                                                                                        new WaitCommand(0.65),
                                                                                        new ParallelCommandGroup(
                                                                                                        new AutoSetForearm(
                                                                                                                        container.forearm,
                                                                                                                        ForearmState.GroundIntakeCube),
                                                                                                        new AutoSetWrist(
                                                                                                                        container.wrist,
                                                                                                                        WristState.AUTO_GroundIntake),
                                                                                                        new AutoSetClaw(container.claw,
                                                                                                                        ClawState.IntakeCube),
                                                                                                        new SequentialCommandGroup(
                                                                                                                        new WaitCommand(1.5),
                                                                                                                        new AutoSetShoulder(
                                                                                                                                        container.shoulder,
                                                                                                                                        ShoulderState.Base1))))),
                                                        new WaitCommand(0.25),
                                                        new AutoWaitForGrab(container.claw),
                                                        new ParallelCommandGroup(
                                                                        new AutoSetClaw(container.claw,
                                                                                        ClawState.IntakeCube),
                                                                        new SequentialCommandGroup(
                                                                                        new AutoSetShoulder(
                                                                                                        container.shoulder,
                                                                                                        ShoulderState.Base2),
                                                                                        new ParallelCommandGroup(
                                                                                                        new AutoSetForearm(
                                                                                                                        container.forearm,
                                                                                                                        ForearmState.AUTO_Base2Cube1),
                                                                                                        new AutoSetWrist(
                                                                                                                        container.wrist,
                                                                                                                        WristState.AUTO_Base2Cube1))),
                                                                        pathFollow(container,
                                                                                        "LeaveComPickUpReturnRightNo180",
                                                                                        new PathConstraints(3, 2))),
                                                        new AutoSetClaw(container.claw,
                                                                        ClawState.FullOuttake),
                                                        new WaitCommand(0.25),
                                                        pathFollow(container, "LeaveComCurveLeft",
                                                                        new PathConstraints(3, 2))
                                        // new WaitCommand(0.5),
                                        // new AutoSetClaw(container.claw, ClawState.FullOuttake),
                                        // new WaitCommand(0.5));
                                        );
                                }),
                PlaceMediumRIGHTBumpMaybePickupCubeBack(
                                "Place Medium from back Pickup Cube (stop if fail) then place Medium from back RIGHT BUMP",
                                (container) -> {
                                        return new SequentialCommandGroup(
                                                        startRecordingError(container),
                                                        new ParallelCommandGroup(
                                                                        new AutoSetForearm(container.forearm,
                                                                                        ForearmState.Base4Cone1),
                                                                        new AutoSetWrist(container.wrist,
                                                                                        WristState.AUTO_Base4Cone1)),
                                                        new WaitCommand(0.5),
                                                        new AutoSetShoulder(container.shoulder, ShoulderState.Base4),
                                                        new WaitCommand(1.0),
                                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                                        new WaitCommand(0.2),

                                                        new ParallelCommandGroup(
                                                                        pathFollow(container,
                                                                                        "LeaveComPickUpLeftNo180",
                                                                                        new PathConstraints(0.66, 1)),
                                                                        new SequentialCommandGroup(
                                                                                        new WaitCommand(0.65),
                                                                                        new ParallelCommandGroup(
                                                                                                        new AutoSetForearm(
                                                                                                                        container.forearm,
                                                                                                                        ForearmState.GroundIntakeCube),
                                                                                                        new AutoSetWrist(
                                                                                                                        container.wrist,
                                                                                                                        WristState.AUTO_GroundIntake),
                                                                                                        new AutoSetClaw(container.claw,
                                                                                                                        ClawState.IntakeCube),
                                                                                                        new SequentialCommandGroup(
                                                                                                                        new WaitCommand(1.5),
                                                                                                                        new AutoSetShoulder(
                                                                                                                                        container.shoulder,
                                                                                                                                        ShoulderState.Base1))))),
                                                        new WaitCommand(0.25),
                                                        new AutoWaitForGrab(container.claw),
                                                        new ParallelCommandGroup(
                                                                        new AutoSetShoulder(container.shoulder,
                                                                                        ShoulderState.Base2),
                                                                        new AutoSetWrist(container.wrist,
                                                                                        WristState.SingleSub)),
                                                        new AutoSetClaw(container.claw,
                                                                        ClawState.IntakeCube),
                                                        new WaitCommand(0.75),
                                                        new AutoSetForearm(container.forearm,
                                                                        ForearmState.SingleSubstation));
                                }),
                PlaceMediumLEFTBumpMaybePickupCubeBack(
                                "Place Medium from back Pickup Cube (stop if fail) then place Medium from back LEFT BUMP",
                                (container) -> {
                                        return new SequentialCommandGroup(
                                                        startRecordingError(container),
                                                        new ParallelCommandGroup(
                                                                        new AutoSetForearm(container.forearm,
                                                                                        ForearmState.Base4Cone1),
                                                                        new AutoSetWrist(container.wrist,
                                                                                        WristState.AUTO_Base4Cone1)),
                                                        new WaitCommand(0.5),
                                                        new AutoSetShoulder(container.shoulder, ShoulderState.Base4),
                                                        new WaitCommand(1.0),
                                                        new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                                                        new WaitCommand(0.2),

                                                        new ParallelCommandGroup(
                                                                        pathFollow(container,
                                                                                        "LeaveComPickUpRightNo180",
                                                                                        new PathConstraints(0.66, 1)),
                                                                        new SequentialCommandGroup(
                                                                                        new WaitCommand(0.65),
                                                                                        new ParallelCommandGroup(
                                                                                                        new AutoSetForearm(
                                                                                                                        container.forearm,
                                                                                                                        ForearmState.GroundIntakeCube),
                                                                                                        new AutoSetWrist(
                                                                                                                        container.wrist,
                                                                                                                        WristState.AUTO_GroundIntake),
                                                                                                        new AutoSetClaw(container.claw,
                                                                                                                        ClawState.IntakeCube),
                                                                                                        new SequentialCommandGroup(
                                                                                                                        new WaitCommand(1.5),
                                                                                                                        new AutoSetShoulder(
                                                                                                                                        container.shoulder,
                                                                                                                                        ShoulderState.Base1))))),
                                                        new WaitCommand(0.25),
                                                        new AutoWaitForGrab(container.claw),
                                                        new ParallelCommandGroup(
                                                                        new AutoSetShoulder(container.shoulder,
                                                                                        ShoulderState.Base2),
                                                                        new AutoSetWrist(container.wrist,
                                                                                        WristState.SingleSub)),
                                                        new AutoSetClaw(container.claw,
                                                                        ClawState.IntakeCube),
                                                        new WaitCommand(0.75),
                                                        new AutoSetForearm(container.forearm,
                                                                        ForearmState.SingleSubstation));
                                });
                // PlaceMediumLEFTPickupSweepCubeBack(
                // "Place Medium from back Sweep to Pickup Cube (stop if fail) then place Medium
                // from back LEFT",
                // (container) -> {
                // return new SequentialCommandGroup(
                // startRecordingError(container),
                // new ParallelCommandGroup(
                // new AutoSetForearm(container.forearm,
                // ForearmState.Base4Cone1),
                // new AutoSetWrist(container.wrist,
                // WristState.AUTO_Base4Cone1)),
                // new WaitCommand(0.5),
                // new AutoSetShoulder(container.shoulder, ShoulderState.Base4),
                // new WaitCommand(1.0),
                // new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                // new WaitCommand(0.2),

                // new ParallelCommandGroup(
                // pathFollow(container,
                // "LeaveComPickUpRightNo180SweepPart1"),
                // new SequentialCommandGroup(
                // new WaitCommand(0.65),
                // new ParallelCommandGroup(
                // new AutoSetForearm(
                // container.forearm,
                // ForearmState.GroundIntakeCube),
                // new AutoSetWrist(
                // container.wrist,
                // WristState.AUTO_GroundIntake),
                // new AutoSetClaw(container.claw,
                // ClawState.IntakeCube),
                // new SequentialCommandGroup(
                // new WaitCommand(1.5),
                // new AutoSetShoulder(
                // container.shoulder,
                // ShoulderState.Base1))))),
                // pathFollow(container,
                // "LeaveComPickUpRightNo180SweepPart2"),
                // new AutoWaitForGrab(container.claw),
                // new WaitCommand(0.25),
                // new ParallelCommandGroup(
                // new AutoSetClaw(container.claw,
                // ClawState.IntakeCube),
                // new SequentialCommandGroup(
                // new AutoSetShoulder(
                // container.shoulder,
                // ShoulderState.Base2),
                // new ParallelCommandGroup(
                // new AutoSetForearm(
                // container.forearm,
                // ForearmState.AUTO_Base2Cube1),
                // new AutoSetWrist(
                // container.wrist,
                // WristState.AUTO_Base2Cube1))),
                // pathFollow(container,
                // "LeaveComPickUpReturnRightNo180",
                // new PathConstraints(3, 2))),
                // new AutoSetClaw(container.claw,
                // ClawState.FullOuttake),
                // new WaitCommand(0.25),
                // pathFollow(container, "LeaveComCurveLeft", new PathConstraints(3, 2))
                // // new WaitCommand(0.5),
                // // new AutoSetClaw(container.claw, ClawState.FullOuttake),
                // // new WaitCommand(0.5));
                // );
                // });

                String displayName;
                Function<RobotContainer, Command> commandFunction;

                Autos(String displayName, Function<RobotContainer, Command> commandFunction) {
                        this.displayName = displayName;
                        this.commandFunction = commandFunction;
                }

        }

        public Command getAutonomousCommand() {
                return Autos.valueOf(autoChooser.getSelected()).commandFunction.apply(this);
        }

        public void initShuffleboardObjects() {
                /*
                 * Path autoPath =
                 * Filesystem.getDeployDirectory().toPath().resolve("pathplanner/");
                 * 
                 * autoChooser = new SendableChooser<>();
                 * 
                 * try (Stream<Path> list = Files.list(autoPath)) {
                 * list.filter(Files::isRegularFile)
                 * .map(Path::getFileName)
                 * .map(Path::toString)
                 * .forEach((String autoFileName) -> {
                 * autoChooser.addOption(autoFileName.split(Pattern.quote("."))[0],
                 * autoFileName.split(Pattern.quote("."))[0]);
                 * });
                 * } catch (IOException e) {
                 * e.printStackTrace();
                 * }
                 * 
                 * autoChooser.setDefaultOption("Choose an Auto!", null);
                 */

                autoChooser = new SendableChooser<>();

                for (Autos auto : Autos.values()) {
                        autoChooser.addOption(auto.displayName, auto.name());
                }

                autoChooser.setDefaultOption("Choose an Auto!", null);

                mainTab = Shuffleboard.getTab("Main");
                fieldTab = Shuffleboard.getTab("Field");
                armTab = Shuffleboard.getTab("Arm");

                mainTab.add(autoChooser).withSize(2, 1).withPosition(0, 0);

                robotPosition = new Field2d();

                robotPosition.setRobotPose(null);

                fieldTab.add(robotPosition).withPosition(1, 0).withSize(7, 4);

                fieldTab.addDouble("Odometry X", () -> drivetrain.getPose().getX()).withPosition(0, 0);
                fieldTab.addDouble("Odometry Y", () -> drivetrain.getPose().getY()).withPosition(0, 1);
                fieldTab.addDouble("Odometry W", () -> drivetrain.getPose().getRotation().getDegrees()).withPosition(0,
                                2);
                fieldTab.addDouble("NavX Pitch", () -> NavX.getPitch()).withPosition(8, 0);
                fieldTab.addDouble("NavX Roll", () -> NavX.getRoll()).withPosition(8, 1);
                fieldTab.addDouble("NavX Yaw", () -> NavX.getYaw()).withPosition(8, 2);

                armTab.addDouble("String Shoulder Position", () -> shoulder.position()).withPosition(0, 1).withSize(2,
                                1);
                armTab.addDouble("Motor Forearm Position", () -> forearm.getAngle()).withPosition(0, 1).withSize(2, 1);
                armTab.addDouble("Real Forearm Position", () -> forearm.getAbsoluteEncoderAngle()).withPosition(2, 1)
                                .withSize(2, 1);
                armTab.addDouble("Motor Wrist Position", () -> wrist.getAngle()).withPosition(0, 2).withSize(2, 1);
                armTab.addDouble("Real Wrist Position", () -> wrist.getAbsoluteEncoderAngle()).withPosition(2, 2)
                                .withSize(2,
                                                1);

                // mainTab.add(camera.getCamera()).withPosition(2, 0).withSize(4, 4);

                mainTab.addDouble("Controller Offset", () -> OI.getDriveOffset()).withPosition(0, 2).withSize(2, 1);
        }

        double previousTime = System.currentTimeMillis() / 1000;
        double armCancelLitAccumulator = 0;

        public void updateShuffleboardObjects() {
                // double currentTime = System.currentTimeMillis() / 1000;
                // double deltaTime = currentTime - previousTime;

                robotPosition.setRobotPose(drivetrain.getPose());
        }

        public void disabledInit() {
                leds.setPattern(LedPattern.IDLE);
        }

        public void teleopInit() {
                drivetrain.unlock();
        }

        public void configureButtonBindings() {
                // OI.resetHeadingEvent().rising().ifHigh(drivetrain::zeroYaw);
                OI.resetHeadingEvent().rising().ifHigh(() -> {
                        OI.zeroControls();
                });

                OI.autoBalanceEvent().rising().ifHigh(() -> {
                        if (alternateAutoBalance)
                                autoBalanceDrivetrainCommand.schedule();
                        else
                                autoBalanceDrivetrainCommand.cancel();

                        alternateAutoBalance = !alternateAutoBalance;
                });

                OI.driveTrainLock().rising().ifHigh(() -> {
                        drivetrain.lock();
                });
                OI.driveTrainLock().falling().ifHigh(() -> {
                        drivetrain.unlock();
                });

                OI.printOdometryEvent().rising().ifHigh(() -> {
                        System.out.println("Current Odo " + drivetrain.getPose().getX() + ":"
                                        + drivetrain.getPose().getY());
                });

                OI.armUndo().rising().ifHigh(() -> arm.undo());

                OI.armStore().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.Store));
                OI.armGroundIntakeCone().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.GroundIntakeCone));
                OI.armGroundIntakeCube().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.GroundIntakeCube));
                OI.armDoubleSubstationCone().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.DoubleSubstationCone));
                OI.armDoubleSubstationCube().rising().ifHigh(() -> {
                        arm.toKeyPosition(ArmPosition.DoubleSubstationCube);
                });
                OI.armSingleSubstation().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.SingleSubstation));
                OI.armBase4Cone2().rising().ifHigh(() -> {
                        arm.toKeyPosition(ArmPosition.Base4Cone2);
                });
                OI.armBase4Cube2().rising().ifHigh(() -> {
                        arm.toKeyPosition(ArmPosition.Base4Cube2);
                });
                OI.armBase4Cone1().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.Base4Cone1));
                OI.armBase4Cube1().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.Base4Cube1));
                OI.armBase2Cone1().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.Base2Cone1));
                OI.armBase2Cube1().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.Base2Cube1));
                OI.armBase1Hybrid().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.Base1Hybrid));

                OI.clawIntakeCone().rising().ifHigh(() -> claw.set(ClawState.IntakeCone));
                OI.clawIntakeCube().rising().ifHigh(() -> claw.set(ClawState.IntakeCube));
                OI.clawOuttakeCone().rising().ifHigh(() -> claw.set(ClawState.OuttakeCone));
                OI.clawOuttakeCube().rising().ifHigh(() -> claw.set(ClawState.OuttakeCube));
                OI.clawIdle().rising().ifHigh(() -> claw.set(ClawState.Idle));
                OI.clawFullPower().rising().ifHigh(() -> claw.set(ClawState.FullOuttake));

                OI.ledsIndicateCone().rising().ifHigh(() -> {
                        // System.out.println(OI.coneMode);
                        leds.setPattern(LedPattern.CONE);
                        OI.coneMode = true;
                });
                OI.ledsIndicateCube().rising().ifHigh(() -> {
                        // System.out.println(OI.coneMode);
                        leds.setPattern(LedPattern.CUBE);
                        OI.coneMode = false;
                });

                OI.toZero().rising().ifHigh(() -> {
                        arm.toZero();
                });
        }
}
