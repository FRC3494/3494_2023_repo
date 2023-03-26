package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
    CANSparkMax clawMotor;

    ClawState currentState;

    Timer stallTimer;

    public Claw() {
        clawMotor = new CANSparkMax(
                Constants.Subsystems.Claw.MOTOR_CHANNEL, MotorType.kBrushless);

        stallTimer = new Timer();

        set(ClawState.Idle);
    }

    public void set(ClawState state) {
        clawMotor.set(Constants.Subsystems.Claw.SPEEDS.get(state));

        currentState = state;
    }

    @Override
    public void periodic() {
        if ((currentState == ClawState.IntakeCone || currentState == ClawState.IntakeCube)
                && clawMotor.getOutputCurrent() >= Constants.Subsystems.Claw.CURRENT_CUTOFF)
            stallTimer.start();
        else {
            stallTimer.stop();
            stallTimer.reset();
        }

        if (stallTimer.hasElapsed(Constants.Subsystems.Claw.CURRENT_CUTOFF_DURATION))
            set(ClawState.Idle);
    }
}
