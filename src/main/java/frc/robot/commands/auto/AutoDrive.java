package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoDrive extends CommandBase {
	Drivetrain drivetrain;
	double xVelocity;
	double yVelocity;
	double rotationVelocity;
	boolean fieldRelative;

	public AutoDrive(Drivetrain drivetrain, double xVelocity, double yVelocity, double rotationVelocity, boolean fieldRelative) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);

		this.xVelocity = xVelocity;
		this.yVelocity = yVelocity;
		this.rotationVelocity = rotationVelocity;
		this.fieldRelative = fieldRelative;
	}

	@Override
	public void execute() {
		drivetrain.drive(xVelocity, yVelocity, rotationVelocity, fieldRelative);
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
