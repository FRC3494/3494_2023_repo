package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoLockDrivetrain extends CommandBase {
	Drivetrain drivetrain;

	public AutoLockDrivetrain(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		drivetrain.lock();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
