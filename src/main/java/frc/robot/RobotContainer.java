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
import frc.robot.subsystems.leds.LedPattern;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.wrist.Wrist;

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

        // autoBalanceDrivetrainCommand = AutoBalanceTeleopGroup.get(drivetrain);

        autoBalanceDrivetrainCommand = AutoBalanceTeleopGroup.get(drivetrain);
        
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
        Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("ArmCone2", new AutoSetArm(arm, ArmPosition.Base4Cone2));
        Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("ArmStore", new AutoSetArm(arm, ArmPosition.Store));
        Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("ClawOpen", new AutoSetClaw(claw, ClawState.IntakeCube));
        Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("ClawClosed",
                new AutoSetClaw(claw, ClawState.OuttakeCube));
        Constants.RobotContainer.PathPlanner.PATH_EVENTS.put("Wait5", new WaitCommand(5));
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
                    new AutoSetClaw(container.claw, ClawState.IntakeCube),
                    new WaitCommand(0.5),
                    new AutoSetArm(container.arm, ArmPosition.Store),
                    pathFollow(container, "Auto1 Segment1"),
                    AutoBalanceGroup.get(container.drivetrain));

        }),
        PlaceThenPark("Place Then Park", (container) -> {
            return new SequentialCommandGroup(
                    new AutoSetArm(container.arm, ArmPosition.Base4Cone2),
                    new AutoSetClaw(container.claw, ClawState.IntakeCube),
                    new WaitCommand(0.5),
                    new AutoSetArm(container.arm, ArmPosition.Store),
                    pathFollow(container, "LeaveCom"));
        }),
        PushExitBalance("Push Cube, Exit, Balance", (container) -> {
            return new SequentialCommandGroup(
                    new AutoSetArm(container.arm, ArmPosition.Store),
                    new WaitCommand(0.5),
                    new AutoSetClaw(container.claw, ClawState.OuttakeCube),
                    pathFollow(container, "Level1ExitPark"),
                    AutoBalanceGroup.get(container.drivetrain));
        }),
        JustBalance("Just Balance", (container) -> {
            return new SequentialCommandGroup(
                    new AutoSetArm(container.arm, ArmPosition.Store),
                    new WaitCommand(0.5),
                    new AutoSetClaw(container.claw, ClawState.OuttakeCube),
                    AutoBalanceGroup.get(container.drivetrain));
        }),
        JustBalanceYAxis("Just Balance Y Axis", (container) -> {
            return new SequentialCommandGroup(
                    new AutoSetArm(container.arm, ArmPosition.Store),
                    new WaitCommand(0.5),
                    new AutoSetClaw(container.claw, ClawState.OuttakeCube),
                    AutoBalanceGroupYAxis.get(container.drivetrain));
        }),
        PlaceThenBalance("Place then Balance Y Axis", (container) -> {
            return new SequentialCommandGroup(
                    new AutoSetArm(container.arm, ArmPosition.Base4Cone2),
                    new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                    new WaitCommand(0.5),
                    new ParallelCommandGroup(
                            new ParallelCommandGroup(
                                new AutoSetArm(container.arm, ArmPosition.Store),
                                new AutoSetClaw(container.claw, ClawState.Idle)),
                            new SequentialCommandGroup(
                                pathFollow(container, "LeaveCom"),
                                new WaitCommand(0.3),
                                AutoBalanceGroupYAxis.get(container.drivetrain))
                            ));
        }),
        PlacePickupBalance("Place then Pickup Cube then Balance", (container) -> {
            return new SequentialCommandGroup(
                    new AutoSetArm(container.arm, ArmPosition.Base4Cone2),
                    new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                    new WaitCommand(0.5),
                    new ParallelCommandGroup(
                            new AutoSetArm(container.arm, ArmPosition.GroundIntakeCube),
                            pathFollow(container, "LeaveComPickUp")
                    ),
                    new WaitCommand(0.2),
                    new AutoSetClaw(container.claw, ClawState.IntakeCone),
                    new WaitCommand(0.5),
                    new ParallelCommandGroup(
                            new AutoSetArm(container.arm, ArmPosition.Base4Cube2),
                            pathFollow(container, "LeaveComPickUpReturn")
                    ),
                    new AutoSetClaw(container.claw, ClawState.OuttakeCone),
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
                    new AutoSetClaw(container.claw, ClawState.OuttakeCone),
                    new WaitCommand(0.5),
                    new ParallelCommandGroup(
                            new AutoSetArm(container.arm, ArmPosition.GroundIntakeCube),
                            pathFollow(container, "LeaveComPickUp")
                    ),
                    new WaitCommand(0.2),
                    new AutoSetClaw(container.claw, ClawState.IntakeCone),
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
        UrMom("ur mom lol", (container) -> pathFollow(container, "ur mom")),
        ArmTest("arm test", (container) -> new SequentialCommandGroup(
            new AutoSetArm(container.arm, ArmPosition.Base2Cube1),
            new WaitCommand(3),
            new AutoSetArm(container.arm, ArmPosition.Base1Hybrid),
            new WaitCommand(3),
            new AutoSetArm(container.arm, ArmPosition.Base2Cube1),
            new WaitCommand(3),
            new AutoSetArm(container.arm, ArmPosition.Store)
        ));

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
        
        fieldTab.addDouble("Forearm Position", () -> forearm.getAngle()).withPosition(8, 3);
        fieldTab.addDouble("Wrist Position", () -> wrist.getAngle()).withPosition(8, 4);


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
            System.out.println("Current Odo " + drivetrain.getPose().getX() + ":" + drivetrain.getPose().getY());
        });

        OI.forearmFineAdjustPositiveEvent().ifHigh(() -> {
            forearm.directDrive(Constants.OI.FOREARM_FINE_ADJUST_SPEED);
        });

        OI.forearmFineAdjustPositiveEvent().rising().ifHigh(() -> forearm.enableDirectDrive());
        OI.forearmFineAdjustPositiveEvent().falling().ifHigh(() -> {
            forearm.directDrive(0);
            forearm.disableDirectDrive();
        });

        OI.forearmFineAdjustNegativeEvent().ifHigh(() -> {
            forearm.directDrive(-Constants.OI.FOREARM_FINE_ADJUST_SPEED);
        });
        OI.forearmFineAdjustNegativeEvent().rising().ifHigh(() -> forearm.enableDirectDrive());
        OI.forearmFineAdjustNegativeEvent().falling().ifHigh(() -> {
            forearm.directDrive(0);
            forearm.disableDirectDrive();
        });
        
        OI.armUndo().rising().ifHigh(() -> arm.undo());
        
        OI.armStore().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.Store));
        OI.armGroundIntakeCone().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.GroundIntakeCone));
        OI.armGroundIntakeCube().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.GroundIntakeCube));
        OI.armDoubleSubstationCone().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.DoubleSubstationCone));
        OI.armDoubleSubstationCube().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.DoubleSubstationCube));
        OI.armSingleSubstation().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.SingleSubstation));
        OI.armBase4Cone2().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.Base4Cone2));
        OI.armBase4Cube2().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.Base4Cube2));
        OI.armBase4Cone1().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.Base4Cone1));
        OI.armBase4Cube1().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.Base4Cube1));
        OI.armBase2Cone1().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.Base2Cone1));
        OI.armBase2Cube1().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.Base2Cube1));
        OI.armBase1Hybrid().rising().ifHigh(() -> arm.toKeyPosition(ArmPosition.Base1Hybrid));
        
        OI.clawIntakeCone().rising().ifHigh(() -> claw.set(ClawState.IntakeCone));
        OI.clawIntakeCube().rising().ifHigh(() -> claw.set(ClawState.IntakeCube));
        OI.clawOuttakeCone().rising().ifHigh(() -> claw.set(ClawState.OuttakeCone));
        OI.clawOuttakeCube().rising().ifHigh(() -> claw.set(ClawState.OuttakeCube));
        OI.clawIdle().rising().ifHigh(()->claw.set(ClawState.Idle));

        OI.ledsIndicateCone().rising().ifHigh(() -> {
            leds.setPattern(LedPattern.CONE);
            OI.coneMode = true;
        });
        OI.ledsIndicateCube().rising().ifHigh(() -> {
            leds.setPattern(LedPattern.CUBE);
            OI.coneMode = false;
        });
    }
}
