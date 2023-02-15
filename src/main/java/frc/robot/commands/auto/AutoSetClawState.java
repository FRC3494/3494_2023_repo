package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawState;

public class AutoSetClawState extends CommandBase{
    private Claw claw;
    private  ClawState clawPosition;
    public AutoSetClawState(Claw claw, ClawState clawState){
        this.claw = claw;
        this.clawPosition = clawState;
    }
    @Override
    public void initialize(){
        claw.set(clawPosition);
    }
}
