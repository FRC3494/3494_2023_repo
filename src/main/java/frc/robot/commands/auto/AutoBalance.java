package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoBalance extends CommandBase {
	public Drivetrain drivetrain;

	double accumulator = 0;

	double balancedTime = 0;

	double previousTime = 0;
	boolean hitPeak = false;

	double lastAngle = 0;

	public AutoBalance(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		previousTime = System.currentTimeMillis() / 1000;
		accumulator = 0;
		balancedTime = 0;
		hitPeak = false;
	}

	@Override
	public void execute() {
		double currentTime = System.currentTimeMillis() / 1000;
		double deltaTime = currentTime - previousTime;
		previousTime = currentTime;

		double currentAngle = -NavX.getPitch();
		// double angleDerivative = lastAngle - currentAngle;
		lastAngle = currentAngle;

		// double angleFactor = Math.min(1 / Math.max(Math.abs(angleDerivative),
		// 0.000001), 10.0) / 4.5;

		if (Math.abs(currentAngle) < Constants.Commands.AutoBalance.LEVEL_ANGLE) {
			setDrivetrain(0, 0, 0, false);

			balancedTime += deltaTime;
		} else {
			double curvedPower = Constants.Commands.AutoBalance.SLOW_POWER * Math.pow(currentAngle / 9, 5);

			setDrivetrain(Math.min(Math.max(curvedPower, -Constants.Commands.AutoBalance.SLOW_POWER),
					Constants.Commands.AutoBalance.SLOW_POWER), 0, 0, false);

			balancedTime = 0;
		}
	}

	@Override
	public boolean isFinished() {
		return balancedTime >= Constants.Commands.AutoBalance.EXIT_TIME;
	}

	public void setDrivetrain(double x, double y, double w, boolean fieldRelative) {
		drivetrain.drive(-x, y, w, fieldRelative);
	}
}
