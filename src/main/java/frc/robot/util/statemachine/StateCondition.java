package frc.robot.util.statemachine;

@FunctionalInterface
public interface StateCondition {
    Boolean check();
}
