package frc.robot.subsystems.shoulder;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.util.statemachine.IStateControllable;

public class Shoulder extends SubsystemBase implements IStateControllable<ArmState> {
    DoubleSolenoid topPiston;

    DoubleSolenoid bottomPiston;

    AnalogPotentiometer stringPotentiometer;

    ShoulderState currentState;

    boolean isDoneMoving = true;
    boolean lastDoneMoving = true;

    public Shoulder() {
        topPiston = new DoubleSolenoid(
                Constants.Subsystems.Pneumatics.BASE_PCM, PneumaticsModuleType.REVPH,
                Constants.Subsystems.Shoulder.TOP_PISTON_SOLENOID_CHANNEL,
                Constants.Subsystems.Shoulder.TOP_PISTON_SOLENOID_CHANNEL + 1);

        bottomPiston = new DoubleSolenoid(
                Constants.Subsystems.Pneumatics.BASE_PCM, PneumaticsModuleType.REVPH,
                Constants.Subsystems.Shoulder.BOTTOM_PISTON_SOLENOID_CHANNEL,
                Constants.Subsystems.Shoulder.BOTTOM_PISTON_SOLENOID_CHANNEL + 1);

        stringPotentiometer = new AnalogPotentiometer(
                Constants.Subsystems.Shoulder.POTENTIOMETER_CHANNEL, 1);

        setState(Constants.Subsystems.Arm.INITIAL_STATE);
    }

    @Override
    public void periodic() {
        if (currentState != null)
            isDoneMoving = isAt(currentState);

        if (isDoneMoving && !lastDoneMoving)
            System.out.println("Shoulder Hit Target");

        lastDoneMoving = isDoneMoving;

        if (isDoneMoving) {
            shouldWeSlow = false;
        }
    }

    void setTopPiston(Value value) {
        topPiston.set((value == Value.kForward) ? Value.kForward : Value.kReverse);
    }

    void setBottomPiston(Value value) {
        bottomPiston.set((value == Value.kReverse) ? Value.kForward
                : Value.kReverse);
    }

    long lastShoulderActuationTime = System.currentTimeMillis();

    public void setState(ArmState newState) {
        setState(newState.shoulderState);
    }

    boolean shouldWeSlow = false;

    public void setState(ShoulderState newState) {
        switch (newState) {
            case Base1:
                setTopPiston(Value.kReverse);
                setBottomPiston(Value.kReverse);

                shouldWeSlow = false;
                break;
            case Base2:
                setTopPiston(Value.kReverse);
                setBottomPiston(Value.kForward);

                shouldWeSlow = false;
                break;
            case Base3:
                setTopPiston(Value.kForward);
                setBottomPiston(Value.kReverse);

                shouldWeSlow = true;
                break;
            case Base4:
                setTopPiston(Value.kForward);
                setBottomPiston(Value.kForward);

                shouldWeSlow = true;
                break;
        }

        if (currentState == newState)
            shouldWeSlow = false;

        currentState = newState;

        lastShoulderActuationTime = System.currentTimeMillis();

        System.out.println("Shoulder: " + newState.toString());
    }

    public boolean isAt(ArmState state) {
        return isAt(state.shoulderState);
    }

    public boolean isAt(ShoulderState state) {
        return Math.abs(position() -
                Constants.Subsystems.Shoulder.POSITIONS
                        .get(state)) <= Constants.Subsystems.Shoulder.TARGET_TOLERANCE;
    }

    public double position() {
        return stringPotentiometer.get();
    }

    public boolean crashDetected() {
        return false;
    }

    public boolean needsSlow() {
        return shouldWeSlow;
    }

    public void toZero() {
        setTopPiston(Value.kReverse);
        setBottomPiston(Value.kForward);
    }
}