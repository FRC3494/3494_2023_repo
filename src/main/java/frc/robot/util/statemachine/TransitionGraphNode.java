package frc.robot.util.statemachine;

public class TransitionGraphNode {
    TransitionGraphNodeType type = TransitionGraphNodeType.Stop;

    TransitionGraphNode trueBranch = null;
    TransitionGraphNode falseBranch = null;
    
    StateBehaviour behaviour = null;
    StateCondition condition = null;

    boolean hasRunOnce = false;

    public TransitionGraphNode() { }

    public TransitionGraphNode(StateBehaviour behaviour, StateCondition finishCondition, TransitionGraphNode next) {
        type = TransitionGraphNodeType.Step;
        this.behaviour = behaviour;
        this.condition = finishCondition;
        this.trueBranch = next;
    }

    public TransitionGraphNode(StateCondition condition, TransitionGraphNode ifTrue, TransitionGraphNode ifFalse) {
        type = TransitionGraphNodeType.Branch;
        this.condition = condition;
        this.trueBranch = ifTrue;
        this.falseBranch = ifFalse;
    }

    public TransitionGraphNode process() {
        switch (type) {
            case Branch:
                if (condition.check()) {
                    if (trueBranch == null) return null;

                    return trueBranch.process();
                }
                
                if (falseBranch == null) return null;

                return falseBranch.process();
            case Step:
                if (!hasRunOnce) behaviour.call();

                hasRunOnce = true;

                if (condition.check()) {
                    hasRunOnce = false;

                    if (trueBranch == null) return null;
                    
                    return trueBranch.process();
                }
        
                return this;
            default:
                break;
        }

        return null;
    }
}
