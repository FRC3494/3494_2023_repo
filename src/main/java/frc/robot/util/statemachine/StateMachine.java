package frc.robot.util.statemachine;

import java.util.ArrayList;
import java.util.List;

public abstract class StateMachine<T extends StateMachineState> {
    public List<StateNode<T>> nodes = new ArrayList<>();

    public int currentNode = 0;

    public List<Integer> queuedNodes = new ArrayList<>();

    public void registerControllables(IStateControllable<?> ...controllables) {

    }

    public void setTarget(T target) {

    }

    public void update() {

    }
}
