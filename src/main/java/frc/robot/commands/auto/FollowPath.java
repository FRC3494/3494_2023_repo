package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FollowPath extends CommandBase {
	Drivetrain drivetrain;
	PathPlannerTrajectory trajectory;

	Timer timer = new Timer();

	Field2d field2d;
	FieldObject2d fieldObject2d;

	public FollowPath(Drivetrain drivetrain, PathPlannerTrajectory trajectory, Field2d field2d) {
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
		drivetrain.resetOdometry(trajectory.getInitialHolonomicPose());

		field2d.setRobotPose(drivetrain.getPose());

        PathPlannerServer.sendActivePath(trajectory.getStates());

		timer.reset();
		timer.start();
	}

	@Override
	public void execute() {
		double curTime = timer.get();

        PathPlannerState targetState = (PathPlannerState) trajectory.sample(curTime); 
        Pose2d targetPose = new Pose2d(targetState.poseMeters.getTranslation(), targetState.holonomicRotation);

        Pose2d currentPose = drivetrain.getPose();

		field2d.setRobotPose(currentPose);
		fieldObject2d.setPose(targetPose);

        double xSpeed = Constants.Commands.FollowPath.X_CONTROLLER.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = Constants.Commands.FollowPath.Y_CONTROLLER.calculate(currentPose.getY(), targetPose.getY());
        double thetaSpeed = Constants.Commands.FollowPath.THETA_CONTROLLER.calculate(currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());

        drivetrain.drive(xSpeed, ySpeed, thetaSpeed, true);

        PathPlannerServer.sendPathFollowingData(targetPose, currentPose);
	}

	@Override
	public boolean isFinished() {
		return trajectory.getTotalTimeSeconds() <= timer.get();
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(0, 0, 0, false);

		timer.stop();
	}
}
