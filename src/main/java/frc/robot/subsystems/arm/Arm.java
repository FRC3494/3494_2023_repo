package frc.robot.subsystems.arm;

import frc.robot.subsystems.forearm.Forearm;
import frc.robot.subsystems.forearm.ForearmState;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperState;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderState;
import frc.robot.util.statemachine.StateMachine;

public class Arm extends StateMachine<ArmState> {
    @SuppressWarnings("unchecked") // java is a hell language
    public Arm(Shoulder shoulder, Forearm forearm, Hopper hopper) {
        super(
            ArmState.every(),
            null,
            new ArmState(ShoulderState.Base2, ForearmState.Store, HopperState.Retracted));

        registerControllables(shoulder, forearm, hopper);
    }
}