package frc.robot.util.statemachine;

public interface IStateControllable<T extends Enum<T>> {
    public void setState(T newState);

    public boolean isAt(T newState);

    public boolean crashDetected();
}
