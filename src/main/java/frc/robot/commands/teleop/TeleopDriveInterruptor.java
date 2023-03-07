package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;

public class TeleopDriveInterruptor extends CommandBase {
	@Override
	public boolean isFinished() {
		if (OI.teleopXVelocity() != 0) return true;
		if (OI.teleopYVelocity() != 0) return true;
		if (OI.teleopTurnVelocity() != 0) return true;
			
		return false;
	}
}
