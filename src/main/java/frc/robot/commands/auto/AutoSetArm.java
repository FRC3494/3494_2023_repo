package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmState;

public class AutoSetArm extends CommandBase {
    private Arm arm;
    private ArmState armState;

    public AutoSetArm(Arm arm, ArmState armState) {
        this.arm = arm;
        this.armState = armState;
        
		addRequirements(arm);
    }

    @Override
    public void initialize() {
        //arm.setArmState(armPosition);
        //arm.setTarget(armPosition);
        arm.setTarget(armState);
    }

    @Override
    public boolean isFinished() {
        return true;
        //return arm.isDoneMoving();
    }
}
