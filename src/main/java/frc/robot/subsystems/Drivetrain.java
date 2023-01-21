package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.ThriftySwerveModuleHelper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
	SwerveModule frontLeft = ThriftySwerveModuleHelper.createNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Front Left Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(0, 0),
			ThriftySwerveModuleHelper.GearRatio.STANDARD,
			Constants.Subsystems.Drivetrain.FrontLeftModule.DRIVE_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.FrontLeftModule.STEER_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.FrontLeftModule.STEER_MOTOR_PORT);

	SwerveModule frontRight = ThriftySwerveModuleHelper.createNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Front Right Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(2, 0),
			ThriftySwerveModuleHelper.GearRatio.STANDARD,
			Constants.Subsystems.Drivetrain.FrontRightModule.DRIVE_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.FrontRightModule.STEER_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.FrontRightModule.STEER_MOTOR_PORT);

	SwerveModule backLeft = ThriftySwerveModuleHelper.createNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Back Left Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(4, 0),
			ThriftySwerveModuleHelper.GearRatio.STANDARD,
			Constants.Subsystems.Drivetrain.BackLeftModule.DRIVE_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.BackLeftModule.STEER_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.BackLeftModule.STEER_MOTOR_PORT);

	SwerveModule backRight = ThriftySwerveModuleHelper.createNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Back Right Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(6, 0),
			ThriftySwerveModuleHelper.GearRatio.STANDARD,
			Constants.Subsystems.Drivetrain.BackRightModule.DRIVE_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.BackRightModule.STEER_MOTOR_PORT,
			Constants.Subsystems.Drivetrain.BackRightModule.STEER_MOTOR_PORT);

	NavX navX;

	// Odometry class for tracking robot pose
	private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
			Constants.Subsystems.Drivetrain.SWERVE_KINEMATICS, 
			getGyroscopeRotation(), 
			getSwerveModulePositions());

	/** Creates a new DriveSubsystem. */
	public Drivetrain() {
	}

	@Override
	public void periodic() {
		odometry.update(getGyroscopeRotation(), getSwerveModulePositions());
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return odometry.getPoseMeters();
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
		odometry.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), pose);
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
		var swerveModuleStates = Constants.Subsystems.Drivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroscopeRotation())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));

		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Subsystems.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);

		if (Math.abs(xSpeed) < .001 && Math.abs(ySpeed) < .001 && Math.abs(rot) < .001) {
			frontLeft.set(0, frontLeft.getState().angle.getRadians());
			frontRight.set(0, frontRight.getState().angle.getRadians());
			backLeft.set(0, backLeft.getState().angle.getRadians());
			backRight.set(0, backRight.getState().angle.getRadians());
			return;
		}

		setModuleStates(swerveModuleStates);
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

		return Rotation2d.fromDegrees(360.0 - NavX.getYaw());
	}

	public void zeroYaw() {
		NavX.zeroYaw();
	}
}
