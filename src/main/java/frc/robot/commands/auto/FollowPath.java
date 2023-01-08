package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FollowPath extends SwerveControllerCommand {
	Drivetrain drivetrain;

	public FollowPath(Drivetrain drivetrain, Trajectory trajectory) {
		super(trajectory,
				drivetrain::getPose,
				drivetrain.getKinematics(),
				Constants.Commands.FollowPath.X_CONTROLLER,
				Constants.Commands.FollowPath.Y_CONTROLLER,
				Constants.Commands.FollowPath.THETA_CONTROLLER,
				drivetrain::setModuleStates,
				drivetrain);

		this.drivetrain = drivetrain;
				
		Constants.Commands.FollowPath.THETA_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
		
		drivetrain.resetOdometry(trajectory.getInitialPose());

		this.andThen(() -> drivetrain.drive(0, 0, 0, false));
	}
}
