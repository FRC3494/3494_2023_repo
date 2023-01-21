package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavX;

public class AutoBalance extends CommandBase {
	public Drivetrain drivetrain;

	double divider = 5;

	double balancedTime = 0;

	double previousTime = 0;

	public AutoBalance(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);
	}

	@Override
  	public void initialize() {
		previousTime = System.currentTimeMillis() / 1000;
  	}

	@Override
	public void execute() {
		double currentTime = System.currentTimeMillis() / 1000;
		double deltaTime = currentTime - previousTime;

		setDrivetrain(-Constants.Commands.AutoBalance.DRIVE_POWER * powerCurve(NavX.getPitch()), 0, 0, true);

		if (Math.abs(NavX.getPitch()) <= Constants.Commands.AutoBalance.LEVEL_ANGLE) {
			divider += Constants.Commands.AutoBalance.DIVIDE_FACTOR;
			
			balancedTime += deltaTime;
		} else {
			balancedTime = 0;
		}

		if (balancedTime >= Constants.Commands.AutoBalance.EXIT_TIME) 
			this.end(false);
	}

  	@Override
  	public void end(boolean interrupted) {
		drivetrain.drive(0, 0, 0, false);
  	}

	public double powerCurve(double x) {
		double correctedPower = Math.pow(NavX.getPitch() / divider, 3);

		return Math.min(Math.max(correctedPower, -1), 1);
	}

	public void setDrivetrain(double x, double y, double w, boolean fieldRelative) {
		drivetrain.drive(x, y, w, fieldRelative);
	}
}
