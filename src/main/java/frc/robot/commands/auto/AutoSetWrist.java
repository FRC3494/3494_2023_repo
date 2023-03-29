package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.forearm.ForearmState;
import frc.robot.subsystems.shoulder.ShoulderState;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristState;

public class AutoSetWrist extends CommandBase {
    private Wrist wrist;
    private WristState wristState;

    public AutoSetWrist(Wrist wrist, WristState wristState) {
        this.wrist = wrist;
        this.wristState = wristState;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setState(new ArmState(ShoulderState.Base2, ForearmState.Store, wristState));
    }

    @Override
    public boolean isFinished() {
        return wrist.isAt(wristState);
    }
}
