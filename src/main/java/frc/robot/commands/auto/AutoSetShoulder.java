package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.forearm.ForearmState;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderState;
import frc.robot.subsystems.wrist.WristState;

public class AutoSetShoulder extends CommandBase {
    private Shoulder shoulder;
    private ShoulderState shoulderState;

    public AutoSetShoulder(Shoulder forearm, ShoulderState forearmState) {
        this.shoulder = forearm;
        this.shoulderState = forearmState;

        addRequirements(forearm);
    }

    @Override
    public void initialize() {
        shoulder.setState(new ArmState(shoulderState, ForearmState.Store, WristState.Store));
    }

    @Override
    public boolean isFinished() {
        return shoulder.isAt(shoulderState);
    }
}
