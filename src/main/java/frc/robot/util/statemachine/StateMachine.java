package frc.robot.util.statemachine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public abstract class StateMachine<T extends StateMachineState> {
    List<T> nodes;
    List<StateConnection<T>> connections;

    HashMap<T, HashMap<T, Boolean>> adjacencyMatrix = new HashMap<>();

    T currentNode;
    T targetNode;

    List<T> history = new ArrayList<>();

    IStateControllable<T>[] controllables;

    public StateMachine(List<T> nodes, List<StateConnection<T>> connections, T initialState) {
        init(nodes, connections, initialState);
    }

    public void init(List<T> nodes, List<StateConnection<T>> connections, T initialState) {
        this.nodes = nodes;
        this.connections = connections;

        setTarget(initialState, true);

        //lord help me
        for (T fromState : nodes) {
            adjacencyMatrix.put(fromState, new HashMap<T, Boolean>());

            for (T toState : nodes) {
                boolean found = false;

                for (StateConnection<T> connection : connections) {
                    if (connection.from.equals(fromState) && connection.to.equals(toState)) {
                        found = true;
                        break;
                    }
                }

                adjacencyMatrix.get(fromState).put(toState, found);
            }
        }
    }

    T preregisteredEquivalent(T similarState) {
        for (T checkingState : nodes) {
            if (checkingState.equals(similarState)) return checkingState;
        }

        return null;
    }

    @SuppressWarnings("unchecked") // java is a hell language
    public void registerControllables(IStateControllable<T> ...controllables) {
        this.controllables = controllables;
    }

    void nextState(T state) {
        currentNode = state;

        for (IStateControllable<T> controllable : controllables) { 
            controllable.setState(state);
        }
    }

    public void setTarget(T target) {
        setTarget(target, false);
    }

    void setTarget(T target, boolean initial) {
        T normalizedState = preregisteredEquivalent(target);

        if (initial) currentNode = normalizedState;
        else history.add(targetNode);

        targetNode = normalizedState;
    }

    public void update() {
        if (currentNode == targetNode) return;

        for (IStateControllable<T> controllable : controllables) { 
            if (controllable.crashDetected()) undo();
        }

        HashMap<T, Integer> distances = new HashMap<>();

        // find the distances!

        if (distances.containsKey(targetNode)) {
            nextState(targetNode);
            
            return;
        }
    }

    public void undo() {
        setTarget(history.remove(history.size() - 1), false);
    }
}
