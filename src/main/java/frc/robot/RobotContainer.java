package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.stream.Stream;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.FollowPath;
import frc.robot.commands.teleop.TeleopDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
	private final Drivetrain drivetrain = new Drivetrain();

	private SendableChooser<String> autoChooser;

	public RobotContainer() {
		// Configure the button bindings
		OI.configureButtonBindings();

		// Add all autos to the auto chooser
		Path autoPath = Filesystem.getDeployDirectory().toPath().resolve("/home/lvuser/deploy/pathplanner/");

		autoChooser = new SendableChooser<>();

		try (Stream<Path> list = Files.list(autoPath)) {
			list.filter(Files::isRegularFile)
				.map(Path::getFileName)
				.map(Path::toString)
				.forEach((String autoFileName) -> {
					autoChooser.addOption(autoFileName, autoFileName);
				});
		} catch (IOException e) {
			e.printStackTrace();
		}

		// Configure default commands
		drivetrain.setDefaultCommand(new TeleopDrive(drivetrain));
	}

	public Command getAutonomousCommand() {
		PathPlannerTrajectory loadedPath = PathPlanner.loadPath(autoChooser.getSelected(), Constants.RobotContainer.PathPlanner.PATH_CONSTRAINTS);

        return new FollowPathWithEvents(
            new FollowPath(drivetrain, loadedPath),
            loadedPath.getMarkers(),
            Constants.RobotContainer.PathPlanner.PATH_EVENTS
        );
	}
}
