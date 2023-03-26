package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.subsystems.forearm.Forearm;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.statemachine.StateMachine;

public class Arm extends StateMachine<ArmState> {
    @SuppressWarnings("unchecked") // java is a hell language
    public Arm(Shoulder shoulder, Forearm forearm, Wrist wrist) {
        super(
            ArmConnections.connections,
            Constants.Subsystems.Arm.INITIAL_STATE);

        registerControllables(shoulder, forearm, wrist);
    }

    public void toKeyPosition(ArmPosition keyPosition) {
        setTarget(Constants.Subsystems.Arm.KEY_POSITIONS.get(keyPosition));
    }
}