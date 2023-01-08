package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends CommandBase {
	Drivetrain drivetrain;

	public TeleopDrive(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		drivetrain.drive(OI.getTeleopXVelocity(), OI.getTeleopYVelocity(), OI.getTeleopTurnVelocity(), true);
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(0, 0, 0, false);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
