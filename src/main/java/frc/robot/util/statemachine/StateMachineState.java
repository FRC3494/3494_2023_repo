package frc.robot.util.statemachine;

public interface StateMachineState {
    public boolean equals(StateMachineState state);

    public Enum<?>[] getEnumArray();
}
