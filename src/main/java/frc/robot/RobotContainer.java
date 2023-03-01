package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.regex.Pattern;
import java.util.stream.Stream;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.RunPneumatics;
import frc.robot.commands.auto.FollowPath;
import frc.robot.commands.groups.AutoBalanceTeleopGroup;
import frc.robot.commands.groups.AutoLineUpTeleopGroup;
import frc.robot.commands.teleop.TeleopDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawState;

public class RobotContainer {
	private final Drivetrain drivetrain;
	private final Pneumatics pneumatics;
	private final Arm arm;
	private final Claw claw;
	private FollowPath followPath;
	private ShuffleboardTab autoTab;
	private ShuffleboardTab fieldTab;

	private SendableChooser<String> autoChooser;

	private Field2d robotPosition;
	
	private Command autoBalanceDrivetrainCommand;
	private boolean alternateAutoBalance = true;
	
	public RobotContainer() {
		NavX.getNavX();
		drivetrain = new Drivetrain();
		pneumatics = new Pneumatics();
		arm = new Arm();
		claw = new Claw();
		
		//autoBalanceDrivetrainCommand = AutoBalanceTeleopGroup.get(drivetrain);

		autoBalanceDrivetrainCommand = AutoBalanceTeleopGroup.get(drivetrain);

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		//drivetrain.setDefaultCommand(new AutoLineUp(drivetrain));
		drivetrain.setDefaultCommand(new TeleopDrive(drivetrain));
		//drivetrain.setDefaultCommand(new driveForward(drivetrain));
		pneumatics.setDefaultCommand(new RunPneumatics(pneumatics));

		initShuffleboardObjects();
	}

	public Command getAutonomousCommand() {
		PathPlannerTrajectory loadedPath = PathPlanner.loadPath(autoChooser.getSelected(), Constants.RobotContainer.PathPlanner.PATH_CONSTRAINTS);
		drivetrain.resetOdometry(loadedPath.getInitialPose());
		robotPosition.setRobotPose(drivetrain.getPose());
		followPath = new FollowPath(drivetrain, loadedPath, robotPosition);

        return new FollowPathWithEvents(
            followPath,
            loadedPath.getMarkers(),
            Constants.RobotContainer.PathPlanner.PATH_EVENTS
        );
	}

	public void initShuffleboardObjects() {
		Path autoPath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/");

		autoChooser = new SendableChooser<>();

		try (Stream<Path> list = Files.list(autoPath)) {
			list.filter(Files::isRegularFile)
				.map(Path::getFileName)
				.map(Path::toString)
				.forEach((String autoFileName) -> {
					autoChooser.addOption(autoFileName.split(Pattern.quote("."))[0], autoFileName.split(Pattern.quote("."))[0]);
				});
		} catch (IOException e) {
			e.printStackTrace();
		}

		autoChooser.setDefaultOption("Choose an Auto!", null);

		autoTab = Shuffleboard.getTab("Autonomous");
		fieldTab = Shuffleboard.getTab("Field");

		autoTab.add(autoChooser).withSize(2, 1);



		robotPosition = new Field2d();

		robotPosition.setRobotPose(null);

		fieldTab.add(robotPosition).withPosition(1, 0).withSize(7, 4);

		fieldTab.addDouble("Odometry X", () -> drivetrain.getPose().getX()).withPosition(0, 0);
		fieldTab.addDouble("Odometry Y", () -> drivetrain.getPose().getY()).withPosition(0, 1);
		fieldTab.addDouble("Odometry W", () -> drivetrain.getPose().getRotation().getDegrees()).withPosition(0, 2);
		fieldTab.addDouble("NavX Pitch", () -> NavX.getPitch()).withPosition(8, 0);
		fieldTab.addDouble("NavX Roll", () -> NavX.getRoll()).withPosition(8, 1);
		fieldTab.addDouble("NavX Yaw", () -> NavX.getYaw()).withPosition(8, 2);
	}

	public void updateShuffleboardObjects() {
		robotPosition.setRobotPose(drivetrain.getPose());
	}

	public void configureButtonBindings() {

		OI.resetHeadingEvent().rising().ifHigh(drivetrain::zeroYaw);
		
		OI.autoBalanceEvent().rising().ifHigh(() -> {
			if (alternateAutoBalance) autoBalanceDrivetrainCommand.schedule();
			else autoBalanceDrivetrainCommand.cancel();
			
			alternateAutoBalance = !alternateAutoBalance;
		});

		OI.autoLineUpEvent().rising().ifHigh(() -> {
			drivetrain.resetOdometry(AutoLineUpTeleopGroup.get(drivetrain, robotPosition));
		});
		
		OI.printOdometryEvent().rising().ifHigh(() -> {
			System.out.println("Current Odo " + drivetrain.getPose().getX() + ":" + drivetrain.getPose().getY());
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
		OI.forearmFineAdjustNegativeEvent().ifHigh(() -> {
			arm.directDriveForearm(-Constants.OI.FOREARM_FINE_ADJUST_SPEED);
		});
		OI.forearmFineAdjustPositiveEvent().or(
			OI.forearmFineAdjustNegativeEvent()
		).rising().ifHigh(() -> {
			arm.enableForearmDirectDrive();
		});
		OI.forearmFineAdjustPositiveEvent().negate().and(
			OI.forearmFineAdjustNegativeEvent().negate()
		).rising().ifHigh(() -> {
			arm.disableForearmDirectDrive();
		});



		OI.armHopperGrab().rising().ifHigh(() -> {
			arm.setArmState(ArmPosition.LowerHopperGrab);
		});

		OI.armGroundIntake().rising().ifHigh(() -> {
			arm.setArmState(ArmPosition.UpperHopperGrab);
		});

		OI.armDoubleSubstation().rising().ifHigh(() -> {
			arm.setArmState(ArmPosition.DoubleSubstation);
		});

		OI.armHybrid().rising().ifHigh(() -> {
			arm.setArmState(ArmPosition.Hybrid);
		});

		OI.armStore().rising().ifHigh(() -> {
			arm.setArmState(ArmPosition.Store);
		});

		OI.armBase4Cone2().rising().ifHigh(() -> {
			arm.setArmState(ArmPosition.Base4Cone2);
		});

		OI.armBase4Cube2().rising().ifHigh(() -> {
			arm.setArmState(ArmPosition.Base4Cube2);
		});

		OI.armBase4Cube1().rising().ifHigh(() -> {
			arm.setArmState(ArmPosition.Base4Cube1);
		});

		OI.armBase2Cone1().rising().ifHigh(() -> {
			arm.setArmState(ArmPosition.Base2Cone1);
		});

		OI.armBase1Cube1().rising().ifHigh(() -> {
			arm.setArmState(ArmPosition.Base1Cube1);
		});
	}
}
