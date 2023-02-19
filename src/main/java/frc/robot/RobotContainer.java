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
import frc.robot.commands.teleop.TeleopDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;

public class RobotContainer {
	private final Drivetrain drivetrain;
	private final Pneumatics pneumatics;
	private final Arm arm;
	private FollowPath followPath;
	private ShuffleboardTab autoTab;
	private ShuffleboardTab fieldTab;

	private SendableChooser<String> autoChooser;

	private Field2d robotPosition;
	private Command autoBalanceDrivetrainCommand;
	private Command autoLineUpDrivetrainCommand;

	private boolean alternateAutoBalance = true;
	private boolean alternateAutoLineUp = true;
	private boolean alternateArmPositions = true;
	
	public RobotContainer() {
		NavX.getNavX();
		drivetrain = new Drivetrain();
		pneumatics = new Pneumatics();
		arm = new Arm();
		
		autoBalanceDrivetrainCommand = AutoBalanceTeleopGroup.get(drivetrain);
		autoLineUpDrivetrainCommand = AutoBalanceTeleopGroup.get(drivetrain);

		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		//drivetrain.setDefaultCommand(new AutoLineUp(drivetrain));
		drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, arm));
		//drivetrain.setDefaultCommand(new driveForward(drivetrain));
		pneumatics.setDefaultCommand(new RunPneumatics(pneumatics));

		initShuffleboardObjects();
	}

	public Command getAutonomousCommand() {
		PathPlannerTrajectory loadedPath = PathPlanner.loadPath(autoChooser.getSelected(), Constants.RobotContainer.PathPlanner.PATH_CONSTRAINTS);

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
		OI.getResetHeadingEvent().rising().ifHigh(drivetrain::zeroYaw);
		
		OI.getAutoBalanceEvent().rising().ifHigh(() -> {
			if (alternateAutoBalance) autoBalanceDrivetrainCommand.schedule();
			else autoBalanceDrivetrainCommand.cancel();
			
			alternateAutoBalance = !alternateAutoBalance;
		});
		OI.getAutoLineUpEvent().rising().ifHigh(() -> {
			if (alternateAutoLineUp) autoLineUpDrivetrainCommand.schedule();
			else autoLineUpDrivetrainCommand.cancel();
			
			alternateAutoLineUp = !alternateAutoLineUp;
		});

		OI.getArmTestButton().rising().ifHigh(() -> {
			if (alternateArmPositions) arm.setArmState(ArmPosition.GroundIntake);
			else arm.setArmState(ArmPosition.Store);
			
			alternateArmPositions = !alternateArmPositions;
		});
	}
}
