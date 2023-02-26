package frc.robot.commands.teleop;

import frc.robot.OI;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.subsystems.Drivetrain;

public class AutoBalanceTeleop extends AutoBalance {
	public AutoBalanceTeleop(Drivetrain drivetrain) {
		super(drivetrain);
	}

	@Override
	public void setDrivetrain(double x, double y, double w, boolean fieldRelative) { 
		drivetrain.drive(x + OI.teleopXVelocity(), OI.teleopYVelocity(), OI.teleopTurnVelocity(), fieldRelative);
	}
}
