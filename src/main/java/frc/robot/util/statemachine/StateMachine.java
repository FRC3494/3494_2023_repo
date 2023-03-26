package frc.robot.util.statemachine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StateMachine<T extends StateMachineState> extends SubsystemBase {
    List<T> nodes;
    List<StateConnection<T>> connections;

    HashMap<T, HashMap<T, Boolean>> adjacencyMatrix = new HashMap<>();

    T currentNode;
    T targetNode;

    List<T> history = new ArrayList<>();

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
        boolean found = false;
        List<T> sequence = new ArrayList<>();
    }

    void recomputeSequence() {
        currentSequence.clear();

        if (currentNode.equals(targetNode)) return;

        List<List<T>> sequences = new ArrayList<>();
        
        for (T node : nodes) {
            if (!currentNode.equals(node) && adjacencyMatrix.get(currentNode).get(node)) {
                if (node.equals(targetNode)) {
                    currentSequence.add(targetNode);
                    sequenceChanged = true;

                    return;
                }

                List<T> seekingSequence = new ArrayList<>();

                SeekResult result = recursiveFind(node, new SeekResult());
    
                if (result.found) {
                    sequences.add(seekingSequence);
                }
            }
        }

        int smallestDistance = Integer.MAX_VALUE;
        int bestSequence = -1;

        for (int i = 0; i < sequences.size(); i++) {
            if (sequences.get(i).size() < smallestDistance)
                bestSequence = i;
        }

        if (bestSequence != -1) {
            currentSequence = sequences.get(bestSequence);
            sequenceChanged = true;
        }
    }

    List<T> nodeHistory = new ArrayList<>();

    SeekResult recursiveFind(T previous, SeekResult seekResult) {
        for (T node : nodes) {
            if (!previous.equals(node) && !seekResult.sequence.contains(node) && adjacencyMatrix.get(previous).get(node)) {
                if (node.equals(targetNode)) {
                    return new SeekResult() {{
                        found = true;

                        sequence = new ArrayList<>();

                        sequence.addAll(seekResult.sequence);
                        sequence.add(node);
                        sequence.add(targetNode);
                    }};
                } else {
                    SeekResult result = recursiveFind(node, new SeekResult() {{
                        found = true;

                        sequence = new ArrayList<>();

                        sequence.addAll(seekResult.sequence);
                        sequence.add(node);
                    }});

                    if (result.found)
                        return result;
                }
            }
        }

        return seekResult;
    }

    public void undo() {
        targetNode = history.remove(history.size() - 1);

        recomputeSequence();
    }
}
