package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavX;

public class AutoBalance extends CommandBase {
	Drivetrain drivetrain;

	NavX navX;

	public AutoBalance(Drivetrain drivetrain, NavX navX) {
		this.drivetrain = drivetrain;
		this.navX = navX;

		addRequirements(drivetrain);
		
		new AutoDrive(drivetrain, 0, -Constants.Commands.AutoBalance.DRIVE_POWER, 0, false)
			.until(() -> Math.abs(navX.getPitch()) >= Constants.Commands.AutoBalance.TRIGGER_ANGLE)
			.andThen(this);
	}

	@Override
  	public void initialize() {
		
  	}

	@Override
	public void execute() {
		drivetrain.drive(0, -Constants.Commands.AutoBalance.DRIVE_POWER * (navX.getPitch() / 10), 0, true);
	}

  	@Override
  	public void end(boolean interrupted) {
		drivetrain.drive(0, 0, 0, false);
  	}
}
