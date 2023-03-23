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
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.LimelightResults;

import com.google.gson.*;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

public class AutoDriveTopt2 extends CommandBase {
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
    
	public AutoDriveTopt2(Drivetrain drivetrain, Field2d field2d) {
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);
		this.field2d = field2d;
	}

	@Override
  	public void initialize() {
		
	}

	@Override
	public void execute() {
		
	}
  	@Override
  	public void end(boolean interrupted) {
		drivetrain.drive(0, 0, 0, false);
  	}
 
	public void setDrivetrain(double x, double y, double w, boolean fieldRelative) {
		drivetrain.drive(x, y, w, fieldRelative);
	}
}
