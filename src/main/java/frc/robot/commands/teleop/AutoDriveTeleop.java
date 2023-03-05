package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveTeleop extends CommandBase {
	Drivetrain drivetrain;
	double xVelocity;
	double yVelocity;
	double rotationVelocity;
	boolean fieldRelative;

	public AutoDriveTeleop(Drivetrain drivetrain, double xVelocity, double yVelocity, double rotationVelocity,
			boolean fieldRelative) {
		this.drivetrain = drivetrain;

		this.xVelocity = xVelocity;
		this.yVelocity = yVelocity;
		this.rotationVelocity = rotationVelocity;
		this.fieldRelative = fieldRelative;

		addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		drivetrain.drive(xVelocity + OI.teleopXVelocity(), yVelocity + OI.teleopYVelocity(),
				rotationVelocity + OI.teleopTurnVelocity(), fieldRelative);
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
