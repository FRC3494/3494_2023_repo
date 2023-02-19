package frc.robot.util.statemachine;

public class TransitionGraph {
    TransitionGraphNode startNode = null;

    TransitionGraphNode currentNode = null;

    public TransitionGraph(TransitionGraphNode startNode) {
        this.startNode = startNode;
    }

    public void prepare() {
        if (startNode == null) return;

        currentNode = startNode;
    }

    public boolean update() {
        if (startNode == null) return true;

        currentNode = currentNode.process();

        if (currentNode == null) return true;

        return false;
    }
}
