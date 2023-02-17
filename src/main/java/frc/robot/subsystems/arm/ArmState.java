package frc.robot.subsystems.arm;

public class ArmState {
    ArmPosition state;
    String substate;

    public ArmState(ArmPosition state, String substate) {
        this.state = state;
        this.substate = substate;
    }
}
