package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class FollowPath extends CommandBase {
	Drivetrain drivetrain;
	PathPlannerTrajectory trajectory;
	HolonomicDriveController controller;
	Timer timer = new Timer();

	Field2d field2d;
	FieldObject2d fieldObject2d;

	boolean fullAutoMode;

	public FollowPath(Drivetrain drivetrain, PathPlannerTrajectory trajectory, Field2d field2d, boolean fullAutoMode) {
		this.drivetrain = drivetrain;
		this.trajectory = trajectory;
		this.field2d = field2d;
		this.fieldObject2d = field2d.getObject("Target Position");

		this.fullAutoMode = fullAutoMode;
		// drivetrain.resetOdometry(trajectory.getInitialPose());


		// drivetrain.resetOdometry(trajectory.getInitialPose());

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		if (fullAutoMode) drivetrain.resetOdometry(trajectory.getInitialHolonomicPose());

		PIDController xController = Constants.Commands.FollowPath.X_CONTROLLER;
		PIDController yController = Constants.Commands.FollowPath.Y_CONTROLLER;
		ProfiledPIDController thetaController = Constants.Commands.FollowPath.THETA_CONTROLLER;

		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		this.controller = new HolonomicDriveController(xController, yController, thetaController);

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
		drivetrain.periodic();
		field2d.setRobotPose(currentPose);
		fieldObject2d.setPose(targetPose);
	
		ChassisSpeeds targetSpeeds = controller.calculate(drivetrain.getPose(), targetState, targetState.holonomicRotation);

		drivetrain.drive(targetSpeeds);
		
        PathPlannerServer.sendPathFollowingData(targetPose, currentPose);
	}

	@Override
	public boolean isFinished() {
		if (!fullAutoMode) return false; 

		return trajectory.getTotalTimeSeconds() <= timer.get();
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(0, 0, 0, false);

		timer.stop();
	}
}
