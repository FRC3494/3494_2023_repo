package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.forearm.Forearm;
import frc.robot.subsystems.forearm.ForearmState;
import frc.robot.subsystems.shoulder.ShoulderState;
import frc.robot.subsystems.wrist.WristState;

public class AutoSetForearm extends CommandBase {
    private Forearm forearm;
    private ForearmState forearmState;

    public AutoSetForearm(Forearm forearm, ForearmState forearmState) {
        this.forearm = forearm;
        this.forearmState = forearmState;

        addRequirements(forearm);
    }

    @Override
    public void initialize() {
        forearm.setState(new ArmState(ShoulderState.Base2, forearmState, WristState.Store));
    }

    @Override
    public boolean isFinished() {
        return forearm.isAt(forearmState);
    }
}
