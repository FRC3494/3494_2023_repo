package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
    CANSparkMax clawMotor;

    ClawState currentState;

    public Claw() {
        clawMotor = new CANSparkMax(
                Constants.Subsystems.Claw.CLAW_MOTOR_CHANNEL, MotorType.kBrushless);

        set(ClawState.Idle);
    }

    public void set(ClawState state) {
        clawMotor.set(Constants.Subsystems.Claw.SPEEDS.get(state));

        currentState = state;
    }

    @Override
    public void periodic() {
        if ((currentState == ClawState.IntakeCone || currentState == ClawState.IntakeCube)
                && clawMotor.getOutputCurrent() >= Constants.Subsystems.Claw.intakeCurrentCutoff)
            set(ClawState.Idle);
    }
}
