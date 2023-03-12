package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FollowPathPP extends PPSwerveControllerCommand {
	Drivetrain drivetrain;
	Trajectory trajectory;

	Timer timer = new Timer();

	Field2d field2d;
	FieldObject2d fieldObject2d;

	public FollowPathPP(Drivetrain drivetrain, PathPlannerTrajectory trajectory, Field2d field2d) {
		super(trajectory,
				drivetrain::getPose,
				drivetrain.getKinematics(),
				Constants.Commands.FollowPath.X_CONTROLLER,
				Constants.Commands.FollowPath.Y_CONTROLLER,
				Constants.Commands.FollowPath.THETA_CONTROLLER,
				drivetrain::setModuleStates,
				false,
				drivetrain);

		this.drivetrain = drivetrain;
		this.trajectory = trajectory;
		this.field2d = field2d;
		this.fieldObject2d = field2d.getObject("Target Position");
		// drivetrain.resetOdometry(trajectory.getInitialPose());
		Constants.Commands.FollowPath.THETA_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);

		// drivetrain.resetOdometry(trajectory.getInitialPose());
	}

	@Override
	public void initialize() {
		drivetrain.resetOdometry(trajectory.getInitialPose());

		field2d.setRobotPose(drivetrain.getPose());

		super.initialize();

		timer.reset();
		timer.start();
	}

	@Override
	public void execute() {
		super.execute();

		double curTime = timer.get();
		field2d.setRobotPose(drivetrain.getPose());
		fieldObject2d.setPose(trajectory.sample(curTime).poseMeters);
	}

	@Override
	public boolean isFinished() {
		return trajectory.getTotalTimeSeconds() <= timer.get();
	}

	@Override
	public void end(boolean interrupted) {
		super.end(interrupted);

		drivetrain.drive(0, 0, 0, false);

		timer.stop();
	}
}
