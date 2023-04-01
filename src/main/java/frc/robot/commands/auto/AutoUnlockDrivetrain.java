package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoUnlockDrivetrain extends CommandBase {
	Drivetrain drivetrain;

	public AutoUnlockDrivetrain(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		drivetrain.unlock();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
