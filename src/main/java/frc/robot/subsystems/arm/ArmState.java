package frc.robot.subsystems.arm;

import frc.robot.subsystems.forearm.ForearmState;
import frc.robot.subsystems.hopper.HopperState;
import frc.robot.subsystems.shoulder.ShoulderState;
import frc.robot.util.statemachine.StateMachineState;

public class ArmState implements StateMachineState {
    public ShoulderState shoulderState;
    public ForearmState forearmState;
    public HopperState hopperState;

    public ArmState(ShoulderState shoulderState, ForearmState forearmState, HopperState hopperState) {
        this.shoulderState = shoulderState;
        this.forearmState = forearmState;
        this.hopperState =  hopperState;
    }

    public boolean equals(StateMachineState otherState) {
        if (!(otherState instanceof ArmState)) return false;

        return shoulderState == ((ArmState) otherState).shoulderState &&
                forearmState == ((ArmState) otherState).forearmState &&
                hopperState == ((ArmState) otherState).hopperState;
    }

    public StateMachineState changesFrom(StateMachineState previousState) {
        return new ArmState(
            (shoulderState == ((ArmState) previousState).shoulderState) ? null : shoulderState,
            (forearmState == ((ArmState) previousState).forearmState) ? null : forearmState,
            (hopperState == ((ArmState) previousState).hopperState) ? null : hopperState
        );
    }
}
