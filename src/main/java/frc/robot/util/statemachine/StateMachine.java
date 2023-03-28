package frc.robot.util.statemachine;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StateMachine<TState extends StateMachineState> extends SubsystemBase {
    List<TState> nodes;
    List<StateConnection<TState>> connections;

    HashMap<TState, HashMap<TState, Double>> adjacencyMatrix = new HashMap<>();

    TState currentNode;
    TState targetNode;

    List<TState> history = new ArrayList<>();
    List<TState> queue = new ArrayList<>();

    List<TState> currentSequence = new ArrayList<>();
    boolean sequenceChanged = false;

    IStateControllable<TState>[] controllables;

    public StateMachine(List<StateConnection<TState>> connections, TState initialState) {
        init(connections, initialState);
    }

    public void init(List<StateConnection<TState>> connections, TState initialState) {
        // this.nodes = nodes;
        this.connections = connections;

        this.nodes = new ArrayList<>();

        for (StateConnection<TState> connection : connections) {
            boolean foundFrom = false;
            boolean foundTo = false;

            for (TState node : nodes) {
                if (node.equals(connection.from))
                    foundFrom = true;
                if (node.equals(connection.to))
                    foundTo = true;
            }

            if (!foundFrom)
                nodes.add(connection.from);
            if (!foundTo && !connection.from.equals(connection.to))
                nodes.add(connection.to);
        }

        setTarget(initialState, true);

        // lord help me
        for (TState fromState : nodes) {
            adjacencyMatrix.put(fromState, new HashMap<TState, Double>());

            for (TState toState : nodes) {
                double distance = -1;

                for (StateConnection<TState> connection : connections) {
                    if (connection.from.equals(fromState) && connection.to.equals(toState)) {
                        distance = 1;
                        break;
                    }
                }

                adjacencyMatrix.get(fromState).put(toState, distance);
            }
        }
    }

    TState preregisteredEquivalent(TState similarState) {
        for (TState checkingState : nodes) {
            if (checkingState.equals(similarState))
                return checkingState;
        }

        return null;
    }

    @SuppressWarnings("unchecked") // java is a hell language
    public void registerControllables(IStateControllable<TState>... controllables) {
        this.controllables = controllables;

        nextState(currentNode);
    }

    void nextState(TState state) {
        currentNode = state;

        for (IStateControllable<TState> controllable : controllables) {
            controllable.setState(state);
        }
    }

    public void setTarget(TState target) {
        setTarget(target, false);
    }

    void setTarget(TState target, boolean initial) {
        TState normalizedState = preregisteredEquivalent(target);

        if (initial)
            currentNode = normalizedState;
        else
            history.add(targetNode);

        targetNode = normalizedState;

        recomputeSequence();
    }

    @Override
    public void periodic() {
        if (currentNode == targetNode)
            return;

        boolean arrived = true;

        for (IStateControllable<TState> controllable : controllables) {
            if (!controllable.isAt(currentNode))
                arrived = false;

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
        List<TState> sequence = new ArrayList<>();
    }

    class SeekContext {
        double distance = 0;
        List<TState> sequence = new ArrayList<>();
    }

    void recomputeSequence() {
        currentSequence.clear();

        if (currentNode.equals(targetNode))
            return;
        /*
         * SeekResult s = new SeekResult();
         * s.sequence = find(currentNode);
         * 
         * SeekResult result = s;
         * System.out.println(result.sequence);
         * // if (result.distance >= 0) {
         * if (result.sequence.size() > 0) {
         * currentSequence = result.sequence;
         * sequenceChanged = true;
         * }
         */

        int nodesLength = nodes.size(); // number of nodes
        int source = nodes.indexOf(currentNode); // source vertex

        LinkedList<Integer> queue = new LinkedList<Integer>();
        boolean used[] = new boolean[nodesLength];
        int d[] = new int[nodesLength];
        int parents[] = new int[nodesLength];

        queue.push(source);
        used[source] = true;
        parents[source] = -1;
        while (!queue.isEmpty()) {
            int v = queue.pop();
            
            for (TState key : adjacencyMatrix.get(nodes.get(v)).keySet()) {
                if (adjacencyMatrix.get(nodes.get(v)).get(key) < 0)
                    continue;

                int u = nodes.indexOf(key);

                if (!used[u]) {
                    used[u] = true;
                    queue.push(u);
                    d[u] = d[v] + 1;
                    parents[u] = v;nodes.get(v);
                }
                
            }
        }

        int targetIndex = nodes.indexOf(targetNode);

        if (!used[targetIndex]) {
            System.out.println("No path!");
        } else {
            ArrayList<Integer> path = new ArrayList<Integer>();

            for (int v = targetIndex; v != -1; v = parents[v])
                path.add(v);

            Collections.reverse(path);

            for (int v : path) {
                if (!nodes.get(v).equals(currentNode))
                    currentSequence.add(nodes.get(v));
            }

            sequenceChanged = true;
        }

        System.out.println("done!");
    }

    ArrayList<TState> find(TState startingNode) {
        for (TState searchingNode : nodes) {
            boolean targetNodeExistsInChildren = adjacencyMatrix.get(startingNode).get(searchingNode) >= 0.0;

            if (targetNodeExistsInChildren) {
                if (searchingNode.equals(targetNode)) {
                    ArrayList<TState> sequence = new ArrayList<TState>();
                    sequence.add(searchingNode);
                    return sequence;
                }
            }

            for (TState searchingNodeB : nodes) {
                if (adjacencyMatrix.get(searchingNode).get(searchingNodeB) >= 0.0) {
                    if (searchingNodeB.equals(targetNode)) {
                        ArrayList<TState> sequence = new ArrayList<TState>();
                        sequence.add(searchingNode);
                        sequence.add(searchingNodeB);
                        return sequence;
                    }
                }
            }
        }
        return new ArrayList<TState>();
    }

    public void undo() {
        targetNode = history.remove(history.size() - 1);

        recomputeSequence();
    }
}
