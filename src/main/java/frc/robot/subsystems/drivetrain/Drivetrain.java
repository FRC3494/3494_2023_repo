package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.List;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.NavX;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Pose2dHelpers;

public class Drivetrain extends SubsystemBase {
	private List<Double> standardDeviationXLeft = new ArrayList<>();
	private List<Double> standardDeviationYLeft = new ArrayList<>();

	private List<Double> standardDeviationXRight = new ArrayList<>();
	private List<Double> standardDeviationYRight = new ArrayList<>();

	public Pose2d limelightBotPoseLeft;
	public Pose2d limelightBotPoseRight;
	public Pose2d averagedPoses;
	public Pose2d limelightBotPoseMaster;
	private boolean resetLeft = false;
	private boolean resetRight = true;

	SwerveModule frontLeft = Mk4iSwerveModuleHelper.createAnalogNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Front Left Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(0, 0),
			Mk4iSwerveModuleHelper.GearRatio.L1,
			Constants.Subsystems.Drivetrain.FrontLeftModule.DRIVE_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.FrontLeftModule.STEER_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.FrontLeftModule.ENCODER_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.FrontLeftModule.STEER_OFFSET);

	SwerveModule frontRight = Mk4iSwerveModuleHelper.createAnalogNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Front Right Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(2, 0),
			Mk4iSwerveModuleHelper.GearRatio.L1,
			Constants.Subsystems.Drivetrain.FrontRightModule.DRIVE_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.FrontRightModule.STEER_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.FrontRightModule.ENCODER_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.FrontRightModule.STEER_OFFSET);

	SwerveModule backLeft = Mk4iSwerveModuleHelper.createAnalogNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Back Left Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(4, 0),
			Mk4iSwerveModuleHelper.GearRatio.L1,
			Constants.Subsystems.Drivetrain.BackLeftModule.DRIVE_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.BackLeftModule.STEER_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.BackLeftModule.ENCODER_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.BackLeftModule.STEER_OFFSET);

	SwerveModule backRight = Mk4iSwerveModuleHelper.createAnalogNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Back Right Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(6, 0),
			Mk4iSwerveModuleHelper.GearRatio.L1,
			Constants.Subsystems.Drivetrain.BackRightModule.DRIVE_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.BackRightModule.STEER_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.BackRightModule.ENCODER_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.BackRightModule.STEER_OFFSET);

	NavX navX;

	// Odometry class for tracking robot pose
	private final SwerveDrivePoseEstimator m_poseEstimator;

	/** Creates a new DriveSubsystem. */
	public Drivetrain() {
		m_poseEstimator = new SwerveDrivePoseEstimator(Constants.Subsystems.Drivetrain.SWERVE_KINEMATICS,
				getGyroscopeRotation(), getSwerveModulePositions(), new Pose2d());
	}

	@Override
	public void periodic() {
		m_poseEstimator.update(getGyroscopeRotation(), getSwerveModulePositions());
		// update limelight position here
		limelightBotPoseLeft = LimelightHelpers.getBotPose2d_wpiBlue("limelight-left");
		limelightBotPoseRight = LimelightHelpers.getBotPose2d_wpiBlue("limelight-right");
		// limelightBotPoseLeft = LimelightHelpers.getBotPose2d("limelight-left");
		// limelightBotPoseRight = LimelightHelpers.getBotPose2d("limelight-right");

		boolean leftNeitherXNorYAt0 = limelightBotPoseLeft.getX() != 0 && limelightBotPoseLeft.getY() != 0;

		limelightBotPoseLeft = new Pose2d(limelightBotPoseLeft.getX(), // + 8.27,
				limelightBotPoseLeft.getY(), /// + 4.01,
				limelightBotPoseLeft.getRotation());

		boolean rightNeitherXNorYAt0 = limelightBotPoseRight.getX() != 0 && limelightBotPoseRight.getY() != 0;
		limelightBotPoseRight = new Pose2d(limelightBotPoseRight.getX(), // + 8.27,
				limelightBotPoseRight.getY(), /// + 4.01,
				limelightBotPoseRight.getRotation());

		// -----------SET MASTER BOT POSE
		if (rightNeitherXNorYAt0 && leftNeitherXNorYAt0 && !RobotState.isAutonomous()) {// resetRight && resetLeft
			averagedPoses = Pose2dHelpers.meanCorrect(limelightBotPoseLeft, limelightBotPoseRight);

			m_poseEstimator.addVisionMeasurement(new Pose2d(averagedPoses.getX(), averagedPoses.getY(),
					getGyroscopeRotation()), Timer.getFPGATimestamp(),
					VecBuilder.fill(0.9, 0.9, 0.9));// taken from soncis squirrls
		} else if (rightNeitherXNorYAt0 && !RobotState.isAutonomous()) {
			m_poseEstimator.addVisionMeasurement(new Pose2d(limelightBotPoseRight.getX(), limelightBotPoseRight.getY(),
					getGyroscopeRotation()),
					Timer.getFPGATimestamp(),
					VecBuilder.fill(0.9, 0.9, 0.9));// taken from soncis squirrls
			// resetOdometry(limelightBotPoseRight);
		} else if (leftNeitherXNorYAt0 && !RobotState.isAutonomous()) {
			// resetOdometry(limelightBotPoseLeft);
			m_poseEstimator.addVisionMeasurement(
					new Pose2d(limelightBotPoseLeft.getX(), limelightBotPoseLeft.getY(),
							getGyroscopeRotation()), // used to be limelightBotPoseLeft.getRotation()
					// new Rotation2d(getGyroscopeRotation().getRadians() + Math.PI)),
					Timer.getFPGATimestamp(),
					VecBuilder.fill(0.9, 0.9, 0.9));// taken from soncis squirrls
		}

		SmartDashboard.putBoolean("Averaging", rightNeitherXNorYAt0 && leftNeitherXNorYAt0);
		SmartDashboard.putNumber("Left Odo", limelightBotPoseLeft.getX());
		SmartDashboard.putNumber("Right Odo", limelightBotPoseRight.getX());
		SmartDashboard.putNumber("True Odo", m_poseEstimator.getEstimatedPosition().getX());
	}

	double nextStandardDeviation(double nextX, double nextY, List<Double> standardDeviationX,
			List<Double> standardDeviationY) {
		standardDeviationX.add(0, nextX);
		standardDeviationY.add(0, nextY);

		if (standardDeviationX.size() >= 11)
			standardDeviationX.remove(standardDeviationX.size() - 1);
		if (standardDeviationY.size() >= 11)
			standardDeviationY.remove(standardDeviationY.size() - 1);

		return Math.pow(
				Math.pow(standardDeviation(standardDeviationX), 2) + Math.pow(standardDeviation(standardDeviationY), 2),
				0.5);
	}

	double standardDeviation(List<Double> list) {
		double acc = 0;
		for (int i = 0; i < list.size(); i++) {
			acc += list.get(i);
		}
		double mean = acc / list.size();
		double midSectionOfTheEquationThatMustBeIterated = 0;
		for (int i = 0; i < list.size(); i++) {
			midSectionOfTheEquationThatMustBeIterated += Math.pow(list.get(i) - mean, 2);
		}
		return Math.pow((midSectionOfTheEquationThatMustBeIterated) / (list.size() - 1), 0.5);

	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_poseEstimator.getEstimatedPosition();
	}

	/**
	 * Returns the current swerve kinematics.
	 *
	 * @return The swerve kinematics.
	 */
	public SwerveDriveKinematics getKinematics() {
		return Constants.Subsystems.Drivetrain.SWERVE_KINEMATICS;
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		m_poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), pose);
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	@SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		if (locked)
			return;

		var swerveModuleStates = Constants.Subsystems.Drivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroscopeRotation())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));

		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
				Constants.Subsystems.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);

		if (Math.abs(xSpeed) < .001 && Math.abs(ySpeed) < .001 && Math.abs(rot) < .001) {
			frontLeft.set(0, frontLeft.getState().angle.getRadians());
			frontRight.set(0, frontRight.getState().angle.getRadians());
			backLeft.set(0, backLeft.getState().angle.getRadians());
			backRight.set(0, backRight.getState().angle.getRadians());
			return;
		}

		setModuleStates(swerveModuleStates);
	}

	public void drive(ChassisSpeeds speeds) {
		SwerveModuleState[] swerveModuleStates = Constants.Subsystems.Drivetrain.SWERVE_KINEMATICS
				.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
				Constants.Subsystems.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
		setModuleStates(swerveModuleStates);
	}

	boolean locked = false;

	public void lock() {
		frontLeft.set(0, Math.toRadians(45));
		frontRight.set(0, -Math.toRadians(45));
		backLeft.set(0, -Math.toRadians(45));
		backRight.set(0, Math.toRadians(45));

		locked = true;
	}

	public void unlock() {
		locked = false;
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
				desiredStates, Constants.Subsystems.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
		frontLeft.set(
				desiredStates[0].speedMetersPerSecond / Constants.Subsystems.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
						* Constants.Subsystems.Drivetrain.MAX_VOLTAGE,
				desiredStates[0].angle.getRadians());
		frontRight.set(
				desiredStates[1].speedMetersPerSecond / Constants.Subsystems.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
						* Constants.Subsystems.Drivetrain.MAX_VOLTAGE,
				desiredStates[1].angle.getRadians());
		backLeft.set(
				desiredStates[2].speedMetersPerSecond / Constants.Subsystems.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
						* Constants.Subsystems.Drivetrain.MAX_VOLTAGE,
				desiredStates[2].angle.getRadians());
		backRight.set(
				desiredStates[3].speedMetersPerSecond / Constants.Subsystems.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
						* Constants.Subsystems.Drivetrain.MAX_VOLTAGE,
				desiredStates[3].angle.getRadians());
	}

	public SwerveModulePosition[] getSwerveModulePositions() {
		return new SwerveModulePosition[] {
				frontLeft.getState(),
				frontRight.getState(),
				backLeft.getState(),
				backRight.getState()
		};
	}

	/**
	 * Returns the current heading of the chassis
	 *
	 * @return The current heading of the chassis.
	 */
	public Rotation2d getGyroscopeRotation() {
		if (NavX.getNavX().isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(NavX.getYaw());
		}
		;

		return Rotation2d.fromDegrees(360.0 - NavX.getYaw());
	}
}
