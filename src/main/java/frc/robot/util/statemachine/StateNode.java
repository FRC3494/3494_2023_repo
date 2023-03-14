package frc.robot.util.statemachine;

import java.util.List;

public class StateNode<T extends StateMachineState> {
    public T state;
    
    List<Integer> connections;

    public StateNode(T state, List<Integer> connections) {
        this.state = state;
        this.connections = connections;
    }
}
