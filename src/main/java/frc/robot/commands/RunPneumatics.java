package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class RunPneumatics extends CommandBase {
	Pneumatics pneumatics;

	public RunPneumatics(Pneumatics pneumatics) {
		this.pneumatics = pneumatics;

		addRequirements(pneumatics);

        //pneumatics.enable();
	}

	@Override
	public void end(boolean interrupted) {
        pneumatics.disable();
	}
}
