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

        int smallestDistance = Integer.MAX_VALUE;
        T smallestTarget = null;

        for (T node : nodes) {
            if (!currentNode.equals(node) && adjacencyMatrix.get(currentNode).get(node)) {
                if (node.equals(targetNode)) {
                    smallestDistance = 0;
                    smallestTarget = node;

                    break;
                }

                nodeHistory.clear();
                nodeHistory.add(currentNode);
                nodeHistory.add(node);
                int result = recursiveFind(node);
    
                if (result >= 0 && result < smallestDistance) {
                    smallestDistance = result;
                    smallestTarget = node;
                }
            }
        }

        if (smallestTarget != null && arrived) {
            nextState(smallestTarget);

            return;
        }
    }

    List<T> nodeHistory = new ArrayList<>();

    @SuppressWarnings("unchecked") // java is a hell language
    int recursiveFind(T previous) {
        for (T node : nodes) {
            if (!previous.equals(node) && !nodeHistory.contains(node) && adjacencyMatrix.get(previous).get(node)) {
                if (node.equals(targetNode)) 
                    return 2;
                else {
                    nodeHistory.add(node);

                    List<T> copiedNodeHistory = (ArrayList<T>) ((ArrayList<T>) nodeHistory).clone();

                    int result = recursiveFind(node);

                    if (result >= 0) 
                        return result + 1;
                    else 
                        nodeHistory = copiedNodeHistory;
                }
            }
        }

        return -1;
    }

    public void undo() {
        setTarget(history.remove(history.size() - 1), false);
    }
}
