package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

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

    public Enum<?>[] getEnumArray() {
        return new Enum<?>[] {
            shoulderState,
            forearmState,
            hopperState
        };
    }

    public static List<ArmState> every() {
        List<ArmState> everyState = new ArrayList<>();

        for (ShoulderState selectedShoulderState : ShoulderState.values()) {
            for (ForearmState selectedForearmState : ForearmState.values()) {
                for (HopperState selectedHopperState : HopperState.values()) {
                    everyState.add(new ArmState(selectedShoulderState, selectedForearmState, selectedHopperState));
                }
            }
        }

        return everyState;
    }
}
