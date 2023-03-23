package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import com.google.gson.*;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

public class AutoLineUp extends CommandBase {
	public Drivetrain drivetrain;

	double divider = 5;

	double balancedTime = 0;

	double previousTime = 0;

	double aprilTagYaw;
	double aprilTagID;  
    JsonObject limeLightData;
	JsonArray limeLightDataArray;
    //static HashMap<String, GenericEntry> tagMap = new HashMap<String, GenericEntry>(); 

	double tagX;
	double tagY;
	double tagZ;

	List<PathPoint> toTagPath;
	Field2d field2d;
	private PathPlannerTrajectory path;
    
	public AutoLineUp(Drivetrain drivetrain, Field2d field2d) {
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);
		this.field2d = field2d;
	}

	@Override
  	public void initialize() {
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
		toTagPath = new ArrayList<PathPoint>(Arrays.asList(new PathPoint(new Translation2d(0,0), new Rotation2d(0)), new PathPoint(new Translation2d(tagX, tagZ), new Rotation2d(0))));
		path = PathPlanner.generatePath(new PathConstraints(Constants.Subsystems.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 2), toTagPath);
		new FollowPath(drivetrain, path, field2d).schedule();
	}

	@Override
	public void execute() {
	}
	public double getStrafePower(double aprilTagYaw){
		double power = isNegative(aprilTagYaw) * Math.pow(Math.abs(aprilTagYaw), 0.5)/5 * Constants.Commands.AutoLineUp.MAX_STRAFE_SPEED;
		if(Math.abs(power)< 0.08){
			power = isNegative(aprilTagYaw) *0.08;
		}
		return power;
	}
	public int isNegative(double number){
		if(number <0){
			return -1;
		}
		else {
			return 1;
		}
	}
  	@Override
  	public void end(boolean interrupted) {
		drivetrain.drive(0, 0, 0, false);
  	}
 
	public void setDrivetrain(double x, double y, double w, boolean fieldRelative) {
		drivetrain.drive(x, y, w, fieldRelative);
	}
}
