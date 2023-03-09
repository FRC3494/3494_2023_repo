package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;

public class TeleopDriveInterruptor extends CommandBase {
	@Override
	public boolean isFinished() {
		if (Math.abs(OI.teleopXVelocity()) >= 0.1 ||
		    Math.abs(OI.teleopYVelocity()) >= 0.1 ||
		    Math.abs(OI.teleopTurnVelocity()) >= 0.1) return true;
			
		return false;
	}
}
