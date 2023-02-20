package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FollowPath extends PPSwerveControllerCommand {
	Drivetrain drivetrain;
	Trajectory trajectory;
	
	Timer timer = new Timer();

	Field2d field2d;
	FieldObject2d fieldObject2d;

	public FollowPath(Drivetrain drivetrain, PathPlannerTrajectory trajectory, Field2d field2d) {
		super(trajectory,
				drivetrain::getPose,
				drivetrain.getKinematics(),
				Constants.Commands.FollowPath.X_CONTROLLER,
				Constants.Commands.FollowPath.Y_CONTROLLER,
				Constants.Commands.FollowPath.THETA_CONTROLLER,
				drivetrain::setModuleStates,
				true,
				drivetrain);

		this.drivetrain = drivetrain;
		this.trajectory = trajectory;
		this.field2d = field2d;
		this.fieldObject2d = field2d.getObject("Target Position");
				
		Constants.Commands.FollowPath.THETA_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
		
		drivetrain.resetOdometry(trajectory.getInitialPose());
	}

	@Override
  	public void initialize() {
		super.initialize();

  		timer.reset();
  		timer.start();
		System.out.println(trajectory.getInitialPose());
		
  	}

	@Override
	public void execute() {
		super.execute();

		double curTime = timer.get();

		fieldObject2d.setPose(trajectory.sample(curTime).poseMeters);
		System.out.println(trajectory.sample(curTime).poseMeters);
	}

  	@Override
  	public void end(boolean interrupted) {
		super.end(interrupted);

		drivetrain.drive(0, 0, 0, false);

  		timer.stop();
  	}
}
