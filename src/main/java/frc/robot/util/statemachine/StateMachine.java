package frc.robot.util.statemachine;

import java.util.ArrayList;
import java.util.List;

public abstract class StateMachine<T extends StateMachineState> {
    public List<StateNode<T>> nodes = new ArrayList<>();

    public List<List<boolean>> adjacencyMatrix = new ArrayList<>();

    public int currentNode = 0;
    public int targetNode = 0;

    public StateMachine() {
        buildTreeFromCurrent(currentNode);
    }

    public void registerControllables(IStateControllable<?> ...controllables) {

    }

    public void setTarget(T target) {
        for (int i = 0; i < nodes.size(); i++) {
            if (nodes[i].equals(target)) {
                targetNode = i;
                break;
            } 
        }
    }

    public void update() {
        if (currentNode == targetNode) return;

    }
}
