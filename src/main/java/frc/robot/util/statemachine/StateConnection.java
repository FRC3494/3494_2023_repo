package frc.robot.util.statemachine;

public class StateConnection<T extends StateMachineState> {
    public T from;
    public T to;
    public int weight = 1;

    public StateConnection(T from, T to) {
        this.from = from;
        this.to = to;

        this.weight = from.distanceTo(to);
    }
}
