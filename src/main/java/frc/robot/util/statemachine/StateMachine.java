package frc.robot.util.statemachine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

public class StateMachine<T extends Enum<T>> {
    protected T currentState;

    protected HashMap<T, List<StateBehaviour>> behaviours = new HashMap<>();
    protected HashMap<T, List<StateBehaviour>> loopedBehaviours = new HashMap<>();
    protected HashMap<T, List<StateBehaviour>> cleanupBehaviours = new HashMap<>();
    protected HashMap<T, HashMap<T, StateCondition>> transitions = new HashMap<>();
    protected HashMap<T, Tuple<T, TransitionGraph>> transitionGraphs = new HashMap<>();

    protected boolean inTransitionEdge = false;
    protected TransitionGraph currentTransitionEdge = null;
    protected T targetState;
    
    /*
     * Allows a subsystem to register a method to be called when the <pre>EventEmitter</pre> transitions to state.
     *
     * @param initialState The initial state for the <pre>EventEmitter</pre> to start in
     */
    public StateMachine(T initialState) {
        currentState = initialState;
        
    }

    /*
     * Allows a subsystem to register a method to be called when the <pre>EventEmitter</pre> transitions to state.
     *
     * @param  state The target state to wait for
     * @param callee The method to call on transition
     */
    public void addBehaviour(T state, StateBehaviour callee) {
        behaviours.computeIfAbsent(state, k -> new ArrayList<>());

        behaviours.get(state).add(callee);
    }

    public void addLoopedBehaviour(T state, StateBehaviour loopedBehaviour) {
        loopedBehaviours.computeIfAbsent(state, k -> new ArrayList<>());

        loopedBehaviours.get(state).add(loopedBehaviour);
    }

    public void addLoopedBehaviour(T state, StateBehaviour loopedBehaviour, StateBehaviour cleanupBehaviour) {
        loopedBehaviours.computeIfAbsent(state, k -> new ArrayList<>());
        cleanupBehaviours.computeIfAbsent(state, k -> new ArrayList<>());

        loopedBehaviours.get(state).add(loopedBehaviour);
        cleanupBehaviours.get(state).add(cleanupBehaviour);
    }

    public void addTransitionGraph(T from, T to, TransitionGraph transitionGraph){
        transitionGraphs.put(to, new Tuple<>(from, transitionGraph));
    }

    /*
     * Allows the code to add a condition to describe a transition between two states.
     *
     * @param activeState The state to wait for before checking <pre>condition</pre>
     * @param targetState The state to transition to when <pre>condition</pre> goes true
     * @param   condition The condition to check before transitioning to <pre>targetState</pre>
     */
    public void setTransitionCondition(T activeState, T targetState, StateCondition condition) {
        transitions.computeIfAbsent(activeState, k -> new HashMap<>());

        transitions.get(targetState).put(activeState, condition);
    }

    /*
     * The function to periodically call in order to allow the <pre>EventEmitter</pre> to make transitions.
     */
    public void update() {
        if (inTransitionEdge) {
            if (!currentTransitionEdge.update()) return;

            inTransitionEdge = false;
            this.targetState = null;
            currentTransitionEdge = null;

            completeTransition(targetState);
        }

        // Check transitions
        HashMap<T, StateCondition> activeTransitions = transitions.get(currentState);

        if (activeTransitions != null) {
            Set<T> conditions = activeTransitions.keySet();

            if (conditions != null) {
                for (T event : conditions) {
                    if (activeTransitions.get(event).check()) {
                        transitionTo(event);
                        return;
                    }
                }
            }
        }

        // Run Looped Behaviours
        if (loopedBehaviours.get(currentState) != null) {
            for (StateBehaviour loopedBehaviour : loopedBehaviours.get(currentState)) {
                loopedBehaviour.call();
            }
        }
    }

    public void transitionTo(T targetState) {
        if (cleanupBehaviours.get(currentState) != null) {
            for (StateBehaviour cleanup : cleanupBehaviours.get(currentState)) {
                cleanup.call();
            }
        }

        if (transitionGraphs.get(targetState) != null) {
            if (transitionGraphs.get(targetState).first == null || transitionGraphs.get(targetState).first == currentState) {
                currentTransitionEdge = transitionGraphs.get(targetState).second;
                this.targetState = targetState;
                inTransitionEdge = true;

                currentTransitionEdge.prepare();

                if (!currentTransitionEdge.update()) return;

                inTransitionEdge = false;
                this.targetState = null;
                currentTransitionEdge = null;
            }
        }

        completeTransition(targetState);
    }

    void completeTransition(T targetState) {
        currentState = targetState;

        if (behaviours.get(targetState) != null) {
            for (StateBehaviour method : behaviours.get(targetState)) {
                try {
                    method.call();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
    }

    // If you read this your cringe why are you reading this stinky library :)
    /*
     * Gets the current state of the <pre>EventEmitter</pre>
     *
     * @returns The current state of the <pre>EventEmitter</pre>
     */
    public T getState() {
        return currentState;
    }
}
