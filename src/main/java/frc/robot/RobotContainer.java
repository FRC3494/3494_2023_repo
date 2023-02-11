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
import frc.robot.commands.auto.AutoLineUp;
import frc.robot.commands.auto.FollowPath;
import frc.robot.commands.groups.AutoBalanceTeleopGroup;
import frc.robot.commands.groups.AutoLineUpTeleopGroup;
import frc.robot.commands.teleop.TeleopDrive;
import frc.robot.commands.teleop.driveForward;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavX;

public class RobotContainer {
	private final Drivetrain drivetrain;

	private FollowPath followPath;
	private boolean alternateAutoBalance = true;
	private boolean alternateAutoLineUp = true;
	private ShuffleboardTab autoTab;
	private ShuffleboardTab fieldTab;

	private SendableChooser<String> autoChooser;

	private Field2d robotPosition;
	private Command autoBalanceDrivetrainCommand;
	private Command autoLineUpDrivetrainCommand;
	
	public RobotContainer() {
		NavX.getNavX();
		drivetrain = new Drivetrain();
		//Constants.InitializeShuffleBoard();
		autoBalanceDrivetrainCommand = AutoBalanceTeleopGroup.get(drivetrain);
		autoLineUpDrivetrainCommand = AutoLineUpTeleopGroup.get(drivetrain);


		// Configure the button bindings
		OI.configureButtonBindings();

		OI.getResetHeadingEvent().rising().ifHigh(drivetrain::zeroYaw);
		
		OI.getAutoBalanceEvent().rising().ifHigh(() -> {
			if (alternateAutoBalance) autoBalanceDrivetrainCommand.schedule();
			else autoBalanceDrivetrainCommand.cancel();
			
			alternateAutoBalance = !alternateAutoBalance;
		});
		OI.getAutoLineUpEvent().rising().ifHigh(
			() -> {
				if (alternateAutoLineUp) autoLineUpDrivetrainCommand.schedule();
				else autoLineUpDrivetrainCommand.cancel();
				
				alternateAutoLineUp = !alternateAutoLineUp;
			});
		

		// Add all autos to the auto chooser
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

		// Configure default commands
		//drivetrain.setDefaultCommand(new AutoLineUp(drivetrain));
		drivetrain.setDefaultCommand(new TeleopDrive(drivetrain));
		//drivetrain.setDefaultCommand(new driveForward(drivetrain));

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

	public Command getAutonomousCommand() {
		PathPlannerTrajectory loadedPath = PathPlanner.loadPath(autoChooser.getSelected(), Constants.RobotContainer.PathPlanner.PATH_CONSTRAINTS);

		followPath = new FollowPath(drivetrain, loadedPath, robotPosition);

        return new FollowPathWithEvents(
            followPath,
            loadedPath.getMarkers(),
            Constants.RobotContainer.PathPlanner.PATH_EVENTS
        );
	}

	public void updateShuffleboardObjects() {
		robotPosition.setRobotPose(drivetrain.getPose());
	}
}
