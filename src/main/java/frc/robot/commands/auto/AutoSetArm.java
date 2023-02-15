package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;

public class AutoSetArm extends CommandBase{
    private Arm arm;
    private ArmPosition armPosition;
    public AutoSetArm(Arm arm, ArmPosition armPosition){
        this.arm = arm;
        this.armPosition = armPosition;
    }
    @Override
    public void initialize(){
        arm.setArmState(armPosition);
    }
}
