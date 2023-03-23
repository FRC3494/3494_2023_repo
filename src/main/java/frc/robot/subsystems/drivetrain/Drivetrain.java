package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.List;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.NavX;
import frc.robot.util.LimelightHelpers;

public class Drivetrain extends SubsystemBase {
	double aprilTagYaw;
	double aprilTagID;  
    JsonObject limeLightData;
	JsonArray limeLightDataArray;
    //static HashMap<String, GenericEntry> tagMap = new HashMap<String, GenericEntry>(); 

	double tagX;
	double tagY;
	double tagZ;
	int i = -1;
	private List<Double> standardDeviationX = new ArrayList<>();
	private List<Double> standardDeviationY = new ArrayList<>();
	public Pose2d limelightBotPose;
	
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

		//update limelight position here
		
		limelightBotPose = LimelightHelpers.getBotPose2d("limelight");
		//System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0));
		//System.out.println("Stdev" + nextStandardDeviation(limelightBotPose.getX(), limelightBotPose.getY()));
		if (nextStandardDeviation(limelightBotPose.getX(), limelightBotPose.getY()) <= Constants.Subsystems.Drivetrain.MAX_STANDARD_DEVIATION_LIMELIGHT && NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0)!=0 && limelightBotPose.getX() != 0 && limelightBotPose.getY() !=0){
			//System.out.println("Sdev" +nextStandardDeviation(limelightBotPose.getX(), limelightBotPose.getY()));

			resetOdometry(new Pose2d(limelightBotPose.getX()+8.27, limelightBotPose.getY()+4.01, limelightBotPose.getRotation()));
			//System.out.println("New Psoe:"+ odometry.getPoseMeters());
		}
	}

	double nextStandardDeviation(double nextX, double nextY){ 
		standardDeviationX.add(0, nextX);
		standardDeviationY.add(0, nextY);

		if (standardDeviationX.size() >= 11) 
			standardDeviationX.remove(standardDeviationX.size() - 1);
		if (standardDeviationY.size() >= 11) 
			standardDeviationY.remove(standardDeviationY.size() - 1);

		return Math.pow(Math.pow(standardDeviation(standardDeviationX), 2) + Math.pow(standardDeviation(standardDeviationY), 2), 0.5);
	}

	double standardDeviation(List<Double> list) {
		double acc = 0;
		for (int i = 0; i < list.size(); i++){
			acc += list.get(i);
		}
		double mean = acc / list.size();
		double midSectionOfTheEquationThatMustBeIterated = 0;
		for (int i = 0; i < list.size(); i++){
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
		if (locked) return;

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

	public void drive(ChassisSpeeds speeds){
		SwerveModuleState[] swerveModuleStates = Constants.Subsystems.Drivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Subsystems.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
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
		};

		return Rotation2d.fromDegrees(360.0 - NavX.getYaw());
	}


	/*public PathPlannerTrajectory getPathToTag(){
		limeLightData = (JsonObject) JsonParser.parseString(
            NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("json")
            .getString("{}")
        );
		limeLightDataArray =  (JsonArray) ((JsonObject) limeLightData.get("Results")).get("Fiducial");
		if(limeLightDataArray.size() != 0){
			System.out.println(limeLightDataArray.get(0).getAsJsonObject().get("fID"));
			System.out.println(limeLightDataArray.get(0).getAsJsonObject().get("tx"));
			aprilTagID =  limeLightDataArray.get(0).getAsJsonObject().get("fID").getAsDouble();
			aprilTagYaw = limeLightDataArray.get(0).getAsJsonObject().get("tx").getAsDouble();
		}
		else{
			aprilTagID = -1;
			aprilTagYaw = 0;
		}
		odometry.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), new Pose2d(tagX, tagZ, getPose().getRotation()));
		System.out.println("Odometery:"+ odometry.getPoseMeters());
		//new PathPoint(new Translation2d(0,0), new Rotation2d(0)),
		ArrayList<PathPoint> toTagPath = new ArrayList<PathPoint>(Arrays.asList(
			new PathPoint(new Translation2d(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY()), new Rotation2d(0)),
			new PathPoint(new Translation2d(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY()+0.1), new Rotation2d(0))));
		
		return PathPlanner.generatePath(new PathConstraints(0.2, 0.2), toTagPath);
	}*/
}
