package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawState;

public class AutoWaitForGrab extends CommandBase {
    private Claw claw;

    public AutoWaitForGrab(Claw claw) {
        this.claw = claw;

        // addRequirements(claw); // don't add the requirement intentionally!
        // you want to be able to use this in parallel with claw operations
    }

    @Override
    public boolean isFinished() {
        return claw.getState() == ClawState.Idle;
    }
}
