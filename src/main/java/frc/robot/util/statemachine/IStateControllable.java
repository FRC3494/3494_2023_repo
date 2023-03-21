package frc.robot.util.statemachine;

public interface IStateControllable<T extends StateMachineState> {
    public void setState(T newState);

    public boolean isAt(T newState);

    public boolean crashDetected();
}
