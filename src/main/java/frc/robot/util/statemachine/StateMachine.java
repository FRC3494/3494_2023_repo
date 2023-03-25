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

        if (!initial) recomputeSequence();
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

    @SuppressWarnings("unchecked") // java is a hell language
    SeekResult recursiveFind(T previous, SeekResult seekResult) {
        for (T node : nodes) {
            if (!previous.equals(node) && !seekResult.sequence.contains(node) && adjacencyMatrix.get(previous).get(node)) {
                if (node.equals(targetNode)) {
                    seekResult.sequence.add(node);
                    seekResult.sequence.add(targetNode);

                    seekResult.found = true;

                    return seekResult;
                } else {
                    seekResult.sequence.add(node);

                    List<T> copiedNodeHistory = (ArrayList<T>) ((ArrayList<T>) seekResult.sequence).clone();

                    SeekResult result = recursiveFind(node, seekResult);

                    if (result.found)
                        return result;
                    else
                        seekResult.sequence = copiedNodeHistory;
                }
            }
        }

        return seekResult;
    }

    public void undo() {
        setTarget(history.remove(history.size() - 1), false);
    }
}
