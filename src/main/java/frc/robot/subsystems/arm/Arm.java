package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.subsystems.forearm.Forearm;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.statemachine.StateMachine;

public class Arm extends StateMachine<ArmState> {
    Forearm forearm;
    Shoulder shoulder;
    Wrist wrist;

    @SuppressWarnings("unchecked") // java is a hell language
    public Arm(Shoulder shoulder, Forearm forearm, Wrist wrist) {
        super(
                ArmConnections.connections,
                Constants.Subsystems.Arm.INITIAL_STATE);

        this.forearm = forearm;
        this.shoulder = shoulder;
        this.wrist = wrist;

        registerControllables(shoulder, forearm, wrist);
    }

    public void toKeyPosition(ArmPosition keyPosition) {
        setTarget(Constants.Subsystems.Arm.KEY_POSITIONS.get(keyPosition));
    }

    @Override
    public void periodic() {

        super.periodic();

        if (shoulder.needsSlow())
            forearm.slowMode(true);
        else
            forearm.slowMode(false);
    }

    public void toZero() {
        shoulder.toZero();
        forearm.toZero();
        wrist.toZero();
    }
}