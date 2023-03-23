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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RunPneumatics;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoSetArm;
import frc.robot.commands.auto.AutoSetClaw;
import frc.robot.commands.auto.FollowPath;
import frc.robot.commands.groups.AutoBalanceGroup;
import frc.robot.commands.groups.AutoBalanceGroupYAxis;
import frc.robot.commands.groups.AutoBalanceTeleopGroup;
import frc.robot.commands.groups.AutoLineUpTeleopGroup;
import frc.robot.commands.teleop.TeleopDrive;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawState;
import frc.robot.subsystems.drivetrain.DriveLocation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.forearm.Forearm;
import frc.robot.subsystems.forearm.ForearmState;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperState;
import frc.robot.subsystems.leds.LedPattern;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderState;

public class RobotContainer {
    public final Drivetrain drivetrain;
    public final Pneumatics pneumatics;
    public final Shoulder shoulder;
    public final Forearm forearm;
    public final Hopper hopper;
    public final Arm arm;
    public final Claw claw;
    public final Leds leds;
    public final Camera camera;
    private ShuffleboardTab mainTab;
    private ShuffleboardTab fieldTab;

    private SendableChooser<String> autoChooser;

    private Field2d robotPosition;

    private Command autoBalanceDrivetrainCommand;
    private boolean alternateAutoBalance = true;
    //private boolean lineUptracker = true;

    public RobotContainer() {
        PathPlannerServer.startServer(3494);

        NavX.getNavX();

        drivetrain = new Drivetrain();

        pneumatics = new Pneumatics();

        shoulder = new Shoulder();
        forearm = new Forearm();
        hopper = new Hopper();

        arm = new Arm(shoulder, forearm, hopper);

        claw = new Claw();

        camera = new Camera();

        leds = new Leds();
        
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // drivetrain.setDefaultCommand(new AutoLineUp(drivetrain));
        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain));
        // drivetrain.setDefaultCommand(new driveForward(drivetrain));
        pneumatics.setDefaultCommand(new RunPneumatics(pneumatics));

        initShuffleboardObjects();

        Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("print", new PrintCommand("Passed print marker"));
        Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("Balance", new AutoBalance(drivetrain));
        Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("ArmCone2", new AutoSetArm(arm, ArmPosition.Base4Cone2));
        Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("ArmStore", new AutoSetArm(arm, ArmPosition.Store));
        Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("ClawOpen", new AutoSetClaw(claw, ClawState.Open));
        Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("ClawClosed",
                new AutoSetClaw(claw, ClawState.Closed));
        Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("Wait5", new WaitCommand(5));

        
        autoBalanceDrivetrainCommand = AutoBalanceTeleopGroup.get(drivetrain);
    }

    public static Command pathFollow(RobotContainer container, String pathName) {
        PathPlannerTrajectory loadedPath = PathPlanner.loadPath(pathName,
                Constants.RobotContainer.PathPlanner.PATH_CONSTRAINTS);
        System.out.println("Running An Auto");
        return new FollowPath(container.drivetrain, loadedPath, container.robotPosition, true);
       /* return new FollowPathWithEvents(new FollowPath(container.drivetrain, loadedPath, container.robotPosition),
                loadedPath.getMarkers(),
                Constants.RobotContainer.PathPlanner.PATH_EVENTS);*/
    }

    public static Command pathFollow(RobotContainer container, String pathName, double targetSpeed) {
        PathPlannerTrajectory loadedPath = PathPlanner.loadPath(pathName, new PathConstraints(targetSpeed,
            Constants.RobotContainer.PathPlanner.PATH_CONSTRAINTS.maxAcceleration));
        return new FollowPath(container.drivetrain, loadedPath, container.robotPosition, true);
        /*return new FollowPathWithEvents(new FollowPath(container.drivetrain, loadedPath, container.robotPosition),
                loadedPath.getMarkers(),
                Constants.RobotContainer.PathPlanner.PATH_EVENTS);*/
    }

    public enum Autos {
        Auto1("Auto1", (container) -> {
            return new SequentialCommandGroup(
                    new AutoSetArm(container.arm, ArmPosition.Base4Cone2),
                    new AutoSetClaw(container.claw, ClawState.Open),
                    new WaitCommand(0.5),
                    new ParallelCommandGroup(
                            new AutoSetArm(container.arm, ArmPosition.Store),
                            new AutoSetClaw(container.claw, ClawState.Closed)),
                    pathFollow(container, "Auto1 Segment1"),
                    AutoBalanceGroup.get(container.drivetrain));

        }),
        PlaceThenPark("Place Then Park", (container) -> {
            return new SequentialCommandGroup(
                    new AutoSetArm(container.arm, ArmPosition.Base4Cone2),
                    new AutoSetClaw(container.claw, ClawState.Open),
                    new WaitCommand(0.5),
                    new ParallelCommandGroup(
                            new AutoSetArm(container.arm, ArmPosition.Store),
                            new AutoSetClaw(container.claw, ClawState.Closed)),
                    pathFollow(container, "LeaveCom"));
        }),
        PushExitBalance("Push Cube, Exit, Balance", (container) -> {
            return new SequentialCommandGroup(
                    new AutoSetArm(container.arm, ArmPosition.LowerHopperGrab),
                    new WaitCommand(0.5),
                    new AutoSetClaw(container.claw, ClawState.Closed),
                    pathFollow(container, "Level1ExitPark"),
                    AutoBalanceGroup.get(container.drivetrain));
        }),
        JustBalance("Just Balance", (container) -> {
            return new SequentialCommandGroup(
                    new AutoSetArm(container.arm, ArmPosition.LowerHopperGrab),
                    new WaitCommand(0.5),
                    new AutoSetClaw(container.claw, ClawState.Closed),
                    AutoBalanceGroup.get(container.drivetrain));
        }),
        JustBalanceYAxis("Just Balance Y Axis", (container) -> {
            return new SequentialCommandGroup(
                    new AutoSetArm(container.arm, ArmPosition.LowerHopperGrab),
                    new WaitCommand(0.5),
                    new AutoSetClaw(container.claw, ClawState.Closed),
                    AutoBalanceGroupYAxis.get(container.drivetrain));
        }),
        PlaceThenBalance("Place then Balance Y Axis", (container) -> {
            return new SequentialCommandGroup(
                    new AutoSetArm(container.arm, ArmPosition.Base4Cone2),
                    new AutoSetClaw(container.claw, ClawState.Open),
                    new WaitCommand(0.5),
                    new ParallelCommandGroup(
                            new ParallelCommandGroup(new AutoSetArm(container.arm, ArmPosition.Store),
                            new AutoSetClaw(container.claw, ClawState.Closed)),
                            new SequentialCommandGroup(
                                pathFollow(container, "LeaveCom"),
                                new WaitCommand(0.3),
                                AutoBalanceGroupYAxis.get(container.drivetrain))
                            ));
        }),
        PlacePickupBalance("Place then Pickup Cube then Balance", (container) -> {
            return new SequentialCommandGroup(
                    new AutoSetArm(container.arm, ArmPosition.Base4Cone2),
                    new AutoSetClaw(container.claw, ClawState.Open),
                    new WaitCommand(0.5),
                    new ParallelCommandGroup(
                            new AutoSetArm(container.arm, ArmPosition.GroundIntake),
                            pathFollow(container, "LeaveComPickUp")
                    ),
                    new WaitCommand(0.2),
                    new AutoSetClaw(container.claw, ClawState.Closed),
                    new WaitCommand(0.5),
                    new ParallelCommandGroup(
                            new AutoSetArm(container.arm, ArmPosition.Base4Cube2),
                            pathFollow(container, "LeaveComPickUpReturn")
                    ),
                    new AutoSetClaw(container.claw, ClawState.Open),
                    pathFollow(container, "Backup")
                    /*new WaitCommand(0.2),
                    new AutoSetClaw(container.claw, ClawState.Open),
                    new WaitCommand(0.5),
                    new AutoSetArm(container.arm, ArmPosition.Store)*/
            );
        }),
        PlacePickupPlace("Place then Pickup Cube then Place", (container) -> {
            return new SequentialCommandGroup(
                    new AutoSetArm(container.arm, ArmPosition.Base4Cone2),
                    new AutoSetClaw(container.claw, ClawState.Open),
                    new WaitCommand(0.5),
                    new ParallelCommandGroup(
                            new AutoSetArm(container.arm, ArmPosition.GroundIntake),
                            pathFollow(container, "LeaveComPickUp")
                    ),
                    new WaitCommand(0.2),
                    new AutoSetClaw(container.claw, ClawState.Closed),
                    new WaitCommand(0.5),
                    pathFollow(container, "FromPickup")
                    /*new WaitCommand(0.2),
                    new AutoSetClaw(container.claw, ClawState.Open),
                    new WaitCommand(0.5),
                    new AutoSetArm(container.arm, ArmPosition.Store)*/
            );
        }),
        Turn90("Turn 90", (container) -> pathFollow(container, "Turn90")),
        Forward2("Forward", (container) -> pathFollow(container, "ForwardX")),
        // Full("Full", (container) -> pathFollow(container, "Full")),
        // ParkTest("Park Test", (container) -> pathFollow(container, "ParkTest")),
        // StarOfDeath("Star of Death", (container) -> pathFollow(container, "Star Of
        // Death")),
        UrMom("ur mom lol", (container) -> pathFollow(container, "ur mom"));

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

        mainTab.add(autoChooser).withSize(2, 1).withPosition(0, 0);

        robotPosition = new Field2d();

        robotPosition.setRobotPose(null);

        fieldTab.add(robotPosition).withPosition(1, 0).withSize(7, 4);

        fieldTab.addDouble("Odometry X", () -> drivetrain.getPose().getX()).withPosition(0, 0);
        fieldTab.addDouble("Odometry Y", () -> drivetrain.getPose().getY()).withPosition(0, 1);
        fieldTab.addDouble("Odometry W", () -> drivetrain.getPose().getRotation().getDegrees()).withPosition(0, 2);
        fieldTab.addDouble("NavX Pitch", () -> NavX.getPitch()).withPosition(8, 0);
        fieldTab.addDouble("NavX Roll", () -> NavX.getRoll()).withPosition(8, 1);
        fieldTab.addDouble("NavX Yaw", () -> NavX.getYaw()).withPosition(8, 2);

        mainTab.add(camera.getCamera()).withPosition(2, 0).withSize(4, 4);

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

    public void testInit() {
        arm.setTarget(new ArmState(ShoulderState.Base4, ForearmState.Base1Cube1, HopperState.Retracted));
    }

    public void configureButtonBindings() {
        OI.armStore().rising().ifHigh(() -> {
            arm.setTarget(Constants.Subsystems.Arm.KEY_POSITIONS.get(ArmPosition.Store));
        });
        OI.armDoubleSubstation().rising().ifHigh(() -> {
            arm.setTarget(Constants.Subsystems.Arm.KEY_POSITIONS.get(ArmPosition.DoubleSubstation));
        });
        OI.armGroundIntake().rising().ifHigh(() -> {
            arm.setTarget(Constants.Subsystems.Arm.KEY_POSITIONS.get(ArmPosition.GroundIntake));
        });
        OI.armBase1Cube1().rising().ifHigh(() -> {
            arm.setTarget(Constants.Subsystems.Arm.KEY_POSITIONS.get(ArmPosition.Base1Cube1));
        });

        /*
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
            System.out.println("Current Odo x" + drivetrain.getPose().getX() + ", y" + drivetrain.getPose().getY());
        });
        /*OI.autoLineUpEvent().rising().ifHigh(() ->{
            AutoLineUpTeleopGroup.get(drivetrain, robotPosition).schedule();
        });*/
        OI.resetMenuLeftGrid().rising().ifHigh(() ->{OI.leftGridMenu = false;});
        OI.resetMenuRightGrid().rising().ifHigh(() ->{OI.rightGridMenu = false;});
        OI.resetMenuMiddleGrid().rising().ifHigh(() ->{OI.middleGridMenu = false;});
        OI.resetMenuPickup().rising().ifHigh(() ->{OI.pickupMenu = false;});

        OI.selectDriveLeftGridMenu().rising().ifHigh(()->{OI.leftGridMenu = true;});
        OI.selectDriveRightGridMenu().rising().ifHigh(()->{OI.rightGridMenu = true;});
        OI.selectDriveMiddleGridMenu().rising().ifHigh(()->{OI.middleGridMenu = true;});
        OI.selectDrivePickupMenu().rising().ifHigh(()->{OI.pickupMenu = true;});

        OI.selectDriveLeftConeLeftGrid().rising().ifHigh(()->{
            OI.leftGridMenu = false;
            AutoLineUpTeleopGroup.go(drivetrain, robotPosition, DriveLocation.LeftConeLeftGrid).schedule();
        });
        OI.selectDriveMiddleCubeLeftGrid().rising().ifHigh(()->{
            OI.leftGridMenu = false;
            AutoLineUpTeleopGroup.go(drivetrain, robotPosition, DriveLocation.MiddleCubeLeftGrid).schedule();
        });
        OI.selectDriveRightConeLeftGrid().rising().ifHigh(()->{
            OI.leftGridMenu = false;
            AutoLineUpTeleopGroup.go(drivetrain, robotPosition, DriveLocation.RightConeLeftGrid).schedule();
        });

        OI.selectDriveLeftConeMiddleGrid().rising().ifHigh(()->{
            OI.middleGridMenu = false;
            AutoLineUpTeleopGroup.go(drivetrain, robotPosition, DriveLocation.LeftConeMiddleGrid).schedule();
        });
        OI.selectDriveMiddleCubeMiddleGrid().rising().ifHigh(()->{
            OI.middleGridMenu = false;
            AutoLineUpTeleopGroup.go(drivetrain, robotPosition, DriveLocation.MiddleCubeMiddleGrid).schedule();
        });
        OI.selectDriveRightConeMiddleGrid().rising().ifHigh(()->{
            OI.middleGridMenu = false;
            AutoLineUpTeleopGroup.go(drivetrain, robotPosition, DriveLocation.RightConeMiddleGrid).schedule();
        });

        OI.selectDriveLeftConeRightGrid().rising().ifHigh(()->{
            OI.rightGridMenu = false;
            AutoLineUpTeleopGroup.go(drivetrain, robotPosition, DriveLocation.LeftConeRightGrid).schedule();
        });
        OI.selectDriveMiddleCubeRightGrid().rising().ifHigh(()->{
            OI.rightGridMenu = false;
            AutoLineUpTeleopGroup.go(drivetrain, robotPosition, DriveLocation.MiddleCubeRightGrid).schedule();
        });
        OI.selectDriveRightConeRightGrid().rising().ifHigh(()->{
            OI.rightGridMenu = false;
            AutoLineUpTeleopGroup.go(drivetrain, robotPosition, DriveLocation.RightConeRightGrid).schedule();
        });

        OI.selectDriveSingleSub().rising().ifHigh(()->{
            OI.pickupMenu = false;
            AutoLineUpTeleopGroup.go(drivetrain, robotPosition, DriveLocation.SingleSubstation).schedule();
        });
        OI.selectDriveDoubleSubLeft().rising().ifHigh(()->{
            OI.pickupMenu = false;
            AutoLineUpTeleopGroup.go(drivetrain, robotPosition, DriveLocation.DoubleSubstationLeft).schedule();
        });
        OI.selectDriveDoubleSubRight().rising().ifHigh(()->{
            OI.pickupMenu = false;
            AutoLineUpTeleopGroup.go(drivetrain, robotPosition, DriveLocation.DoubleSubstationRight).schedule();

        });

        OI.zeroArm().rising().ifHigh(() -> {
            arm.declareInStore();
        });

        OI.clawOpenEvent().rising().ifHigh(() -> {
            claw.set(ClawState.Closed);
        });

        OI.clawCloseEvent().rising().ifHigh(() -> {
            claw.set(ClawState.Open);
        });

        OI.forearmFineAdjustPositiveEvent().ifHigh(() -> {
            arm.directDriveForearm(Constants.OI.FOREARM_FINE_ADJUST_SPEED);
        });
        OI.forearmFineAdjustPositiveEvent().rising().ifHigh(() -> arm.enableForearmDirectDrive());
        OI.forearmFineAdjustPositiveEvent().falling().ifHigh(() -> {
            arm.directDriveForearm(0);
            arm.disableForearmDirectDrive();
        });

        OI.forearmFineAdjustNegativeEvent().ifHigh(() -> {
            arm.directDriveForearm(-Constants.OI.FOREARM_FINE_ADJUST_SPEED);
        });
        OI.forearmFineAdjustNegativeEvent().rising().ifHigh(() -> arm.enableForearmDirectDrive());
        OI.forearmFineAdjustNegativeEvent().falling().ifHigh(() -> {
            arm.directDriveForearm(0);
            arm.disableForearmDirectDrive();
        });

        OI.shoulderBase1().rising().ifHigh(() -> {
            if (!arm.isInCancelMode())
                return;

            arm.setShoulderState(ShoulderState.Base1);
        });
        OI.shoulderBase2().rising().ifHigh(() -> {
            if (!arm.isInCancelMode())
                return;

            arm.setShoulderState(ShoulderState.Base2);
        });
        OI.shoulderBase4().rising().ifHigh(() -> {
            if (!arm.isInCancelMode())
                return;

            arm.setShoulderState(ShoulderState.Base4);
        });

        OI.hopperExtend().rising().ifHigh(() -> {
            arm.setHopperState(HopperState.Extended);
        });

        OI.hopperExtend().falling().ifHigh(() -> {
            if (arm.isInCancelMode())
                return;

            arm.setHopperState(HopperState.Retracted);
        });

        OI.hopperRetract().rising().ifHigh(() -> {
            if (arm.isInCancelMode())
                arm.setHopperState(HopperState.Retracted);
        });

        OI.armCancelToggle().rising().ifHigh(() -> {
            if (!arm.isInCancelMode()) {
                arm.startCancelMode();
            } else {
                arm.endCancelMode();
            }
        });

        OI.armHopperGrab().rising().ifHigh(() -> {
            if (arm.isInCancelMode())
                return;

            arm.setArmState(ArmPosition.LowerHopperGrab);
        });

        OI.armGroundIntake().rising().ifHigh(() -> {
            if (arm.isInCancelMode())
                return;

            arm.setArmState(ArmPosition.GroundIntake);
        });

        OI.armDoubleSubstation().rising().ifHigh(() -> {
            if (arm.isInCancelMode())
                return;

            arm.setArmState(ArmPosition.DoubleSubstation);
        });

        OI.armHybrid().rising().ifHigh(() -> {
            if (arm.isInCancelMode())
                return;

            arm.setArmState(ArmPosition.Hybrid);
        });

        OI.armStore().rising().ifHigh(() -> {
            if (arm.isInCancelMode())
                return;

            arm.setArmState(ArmPosition.Store);
        });

        OI.armBase4Cone2().rising().ifHigh(() -> {
            if (arm.isInCancelMode())
                return;

            arm.setArmState(ArmPosition.Base4Cone2);
        });

        OI.armBase4Cube2().rising().ifHigh(() -> {
            if (arm.isInCancelMode())
                return;

            arm.setArmState(ArmPosition.Base4Cube2);
        });

        OI.armBase4Cube1().rising().ifHigh(() -> {
            if (arm.isInCancelMode())
                return;

            arm.setArmState(ArmPosition.Base4Cube1);
        });

        OI.armBase2Cone1().rising().ifHigh(() -> {
            if (arm.isInCancelMode())
                return;

            arm.setArmState(ArmPosition.Base2Cone1);
        });

        OI.armBase1Cube1().rising().ifHigh(() -> {
            if (arm.isInCancelMode())
                return;

            arm.setArmState(ArmPosition.Base1Cube1);
        });

        OI.ledsIndicateCone().rising().ifHigh(() -> leds.setPattern(LedPattern.CONE));
        OI.ledsIndicateCube().rising().ifHigh(() -> leds.setPattern(LedPattern.CUBE));
        */
    }
}
