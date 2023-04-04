package frc.robot.util.statemachine;

public interface StateMachineState {
    public boolean equals(StateMachineState state);

    public Enum<?>[] getEnumArray();

    public int distanceTo(StateMachineState state);
}
