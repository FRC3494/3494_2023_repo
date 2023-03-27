package frc.robot.util.statemachine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StateMachine<T extends StateMachineState> extends SubsystemBase {
    List<T> nodes;
    List<StateConnection<T>> connections;

    HashMap<T, HashMap<T, Double>> adjacencyMatrix = new HashMap<>();

    T currentNode;
    T targetNode;

    List<T> history = new ArrayList<>();
    List<T> queue = new ArrayList<>();

    List<T> currentSequence = new ArrayList<>();
    boolean sequenceChanged = false;

    IStateControllable<T>[] controllables;

    public StateMachine(List<StateConnection<T>> connections, T initialState) {
        init(connections, initialState);
    }

    public void init(List<StateConnection<T>> connections, T initialState) {
        //this.nodes = nodes;
        this.connections = connections;

        this.nodes = new ArrayList<>();

        for (StateConnection<T> connection : connections) {
            boolean foundFrom = false;
            boolean foundTo = false;

            for (T node : nodes) {
                if (node.equals(connection.from)) foundFrom = true;
                if (node.equals(connection.to)) foundTo = true;
            }

            if (!foundFrom) nodes.add(connection.from);
            if (!foundTo && !connection.from.equals(connection.to)) nodes.add(connection.to);
        }

        setTarget(initialState, true);

        //lord help me
        for (T fromState : nodes) {
            adjacencyMatrix.put(fromState, new HashMap<T, Double>());

            for (T toState : nodes) {
                double distance = -1;

                for (StateConnection<T> connection : connections) {
                    if (connection.from.equals(fromState) && connection.to.equals(toState)) {
                        distance = 1;
                        break;
                    }
                }

                adjacencyMatrix.get(fromState).put(toState, distance);
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

        nextState(currentNode);
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

        recomputeSequence();
    }

    @Override
    public void periodic() {
        if (currentNode == targetNode) return;

        boolean arrived = true;

        for (IStateControllable<T> controllable : controllables) { 
            if (!controllable.isAt(currentNode)) arrived = false;

            if (controllable.crashDetected()) {
                undo();
                arrived = true;
                break;
            }
        }

        if (sequenceChanged) {
            arrived = true;
            sequenceChanged = false;
        }

        if (arrived && currentSequence.size() != 0) {
            nextState(currentSequence.remove(0));

            return;
        }
    }

    class SeekResult {
        double distance = -1;
        List<T> sequence = new ArrayList<>();
    }

    class SeekContext {
        double distance = 0;
        List<T> sequence = new ArrayList<>();
    }

    void recomputeSequence() {
        currentSequence.clear();

        if (currentNode.equals(targetNode)) return;
        
        SeekResult result = seekFrom(currentNode, new SeekContext() {{
            sequence.add(currentNode);
        }});

        if (result.distance >= 0) {
            currentSequence = result.sequence;
            sequenceChanged = true;
        }
    }

    boolean seekFrom(T node, SeekContext seekContext) {
        if (nodes.contains(targetNode)) {
            queue.push(targetNode);
            queue.push(node);
            return true;
}
        

        if (!seekContext.sequence.contains(possibleNode)) {
            SeekResult option = seekFrom(possibleNode, new SeekContext() {{
                distance = seekContext.distance + possibleDistance;

                sequence.addAll(seekContext.sequence);
                sequence.add(possibleNode);
            }});
        }
    }

    public void undo() {
        targetNode = history.remove(history.size() - 1);

        recomputeSequence();
    }
}
