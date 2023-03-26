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

    void setTopPiston(Value value) {
        topPiston.set((value == Value.kForward) ? Value.kForward : Value.kReverse);
    }

    void setBottomPiston(Value value) {
        bottomPiston.set((value == Value.kReverse) ? Value.kForward
                : Value.kReverse);
    }

    long lastShoulderActuationTime = System.currentTimeMillis();

    public void setState(ArmState newState) {
        switch (newState.shoulderState) {
            case Base1:
                setTopPiston(Value.kReverse);
                setBottomPiston(Value.kReverse);
                break;
            case Base2:
                setTopPiston(Value.kReverse);
                setBottomPiston(Value.kForward);
                break;
            case Base3:
                setTopPiston(Value.kForward);
                setBottomPiston(Value.kReverse);
                break;
            case Base4:
                setTopPiston(Value.kForward);
                setBottomPiston(Value.kForward);
                break;
        }

        currentState = newState.shoulderState;

        lastShoulderActuationTime = System.currentTimeMillis();

        System.out.println("Shoulder: " + newState.toString());
    }

    public boolean isAt(ArmState state) {
        return Math.abs(stringPotentiometer.get() -
                Constants.Subsystems.Shoulder.POSITIONS
                        .get(state.shoulderState)) <= Constants.Subsystems.Shoulder.TARGET_TOLERANCE;
    }

    public boolean crashDetected() {
        return false;
    }
}