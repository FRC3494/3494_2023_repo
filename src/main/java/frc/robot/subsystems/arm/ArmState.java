package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants;
import frc.robot.subsystems.forearm.ForearmState;
import frc.robot.subsystems.shoulder.ShoulderState;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.util.statemachine.StateMachineState;

public class ArmState implements StateMachineState {
    public ShoulderState shoulderState;
    public ForearmState forearmState;
    public WristState wristState;

    public ArmState(ShoulderState shoulderState, ForearmState forearmState, WristState wristState) {
        this.shoulderState = shoulderState;
        this.forearmState = forearmState;
        this.wristState = wristState;
    }

    public ArmState(ShoulderState shoulderState, WristState wristState, ForearmState forearmState) {
        this.shoulderState = shoulderState;
        this.forearmState = forearmState;
        this.wristState = wristState;
    }

    public boolean equals(StateMachineState otherState) {
        ArmState armState = (ArmState) otherState;

        return shoulderState == armState.shoulderState &&
                forearmState == armState.forearmState &&
                wristState == armState.wristState;
    }

    public Enum<?>[] getEnumArray() {
        return new Enum<?>[] {
                shoulderState,
                forearmState,
                wristState
        };
    }

    public static List<ArmState> every() {
        List<ArmState> everyState = new ArrayList<>();

        for (ShoulderState selectedShoulderState : ShoulderState.values()) {
            for (ForearmState selectedForearmState : ForearmState.values()) {
                for (WristState selectedWristState : WristState.values()) {
                    everyState.add(new ArmState(selectedShoulderState, selectedForearmState, selectedWristState));
                }
            }
        }

        return everyState;
    }

    public String toString() {
        return "[ " + shoulderState.name() + ", " + forearmState.name() + ", " + wristState.name() + "]";
    }

    public int distanceTo(StateMachineState state) {
        ArmState armState = (ArmState) state;

        double shoulderDelta = Math
                .abs(Constants.Subsystems.Shoulder.POSITIONS.get(this.shoulderState)
                        - Constants.Subsystems.Shoulder.POSITIONS.get(armState.shoulderState))
                * Constants.Subsystems.Shoulder.RADIUS;

        double forearmDelta = Math
                .abs(Constants.Subsystems.Forearm.POSITIONS.get(this.forearmState)
                        - Constants.Subsystems.Forearm.POSITIONS.get(armState.forearmState))
                * Constants.Subsystems.Forearm.RADIUS;

        double wristDelta = Math
                .abs(Constants.Subsystems.Wrist.POSITIONS.get(this.wristState)
                        - Constants.Subsystems.Wrist.POSITIONS.get(armState.wristState))
                * Constants.Subsystems.Wrist.RADIUS;

        return (int) Math.round((shoulderDelta + forearmDelta + wristDelta) * 1000);
    }
}
