package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawState;

public class AutoSetClaw extends CommandBase {
    private Claw claw;
    private ClawState clawPosition;

    public AutoSetClaw(Claw claw, ClawState clawState) {
        this.claw = claw;
        this.clawPosition = clawState;

		addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.set(clawPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
