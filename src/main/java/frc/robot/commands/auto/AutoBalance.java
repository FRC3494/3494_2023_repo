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
	boolean hitPeak = false;

	public AutoBalance(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		previousTime = System.currentTimeMillis() / 1000;
		divider = 5;
		balancedTime = 0;
	}

	@Override
	public void execute() {
		double currentTime = System.currentTimeMillis() / 1000;
		double deltaTime = currentTime - previousTime;
		
		if (Math.abs(NavX.getRoll()) > 20) {
			hitPeak = true;
		} else if (Math.abs(NavX.getRoll()) < Constants.Commands.AutoBalance.TRIGGER_ANGLE && hitPeak) {
			setDrivetrain(-Constants.Commands.AutoBalance.SLOW_POWER, 0, 0, false);
		} else if (Math.abs(NavX.getRoll()) < 5 && hitPeak) {
			setDrivetrain(0, 0, 0, false);

		} else {
			setDrivetrain(Constants.Commands.AutoBalance.FAST_POWER * staticPowerCurve(-NavX.getRoll() / 3), 0, 0,
					false);
		}

		/*
		 * if (Math.abs(NavX.getRoll()) <= Constants.Commands.AutoBalance.DIVIDE_ANGLE)
		 * {
		 * divider += Constants.Commands.AutoBalance.DIVIDE_FACTOR;
		 * // setDrivetrain(0, 0, 0.01, false);
		 * balancedTime += deltaTime;
		 * }
		 * 
		 * else {
		 * balancedTime = 0;
		 * }
		 */
	}

	@Override
	public boolean isFinished() {
		return false;
		// return balancedTime >= Constants.Commands.AutoBalance.EXIT_TIME;
	}

	public double powerCurve(double x) {
		double correctedPower = Math.pow(NavX.getRoll() / divider, 3);

		return Math.min(Math.max(correctedPower, -1), 1);
	}

	public double staticPowerCurve(double x) {
		double correctedPower = Math.pow(NavX.getRoll() / 5, 3);

		return Math.min(Math.max(correctedPower, -1), 1);
	}

	public void setDrivetrain(double x, double y, double w, boolean fieldRelative) {
		drivetrain.drive(x, y, w, fieldRelative);
	}
}
