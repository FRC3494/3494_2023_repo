package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.util.statemachine.IStateControllable;

public class Wrist extends SubsystemBase implements IStateControllable<ArmState> {
    private CANSparkMax motor;

    SparkMaxAbsoluteEncoder encoder;

    WristState currentState;

    boolean isDoneMoving = true;

    public Wrist() {
        motor = new CANSparkMax(
                Constants.Subsystems.Wrist.MOTOR_CHANNEL, MotorType.kBrushless);

        motor.getPIDController().setOutputRange(-0.18, 0.18);//was 0.5
        motor.getPIDController().setP(Constants.Subsystems.Wrist.PIDF.P);
        motor.getPIDController().setI(Constants.Subsystems.Wrist.PIDF.I);
        motor.getPIDController().setD(Constants.Subsystems.Wrist.PIDF.D);
        motor.getPIDController().setFF(Constants.Subsystems.Wrist.PIDF.F);

        motor.setClosedLoopRampRate(0.5);

        motor.setSmartCurrentLimit(10);
        motor.setIdleMode(IdleMode.kBrake);

        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        encoder.setPositionConversionFactor(360);

        correctNeo();

        setState(Constants.Subsystems.Arm.INITIAL_STATE);
    }

    @Override
    public void periodic() {
        System.out.println("w a" + getAbsoluteEncoderAngle() + " m" + getAngle());

        if (currentState != null)
            isDoneMoving = isAt(currentState);
    }

    double getAbsoluteEncoderAngle() {
        return -encoder.getPosition() + 180;
    }

    public double getAngle() {
        return motor.getEncoder().getPosition() *
                Constants.Subsystems.Wrist.MOTOR_REDUCTION * 360.0;
    }

    void correctNeo() {
        motor.getEncoder().setPosition(
                (getAbsoluteEncoderAngle() / 360.0) /
                        Constants.Subsystems.Wrist.MOTOR_REDUCTION);
    }

    public void setState(ArmState newState) {
        setTargetAngle(
                Constants.Subsystems.Wrist.POSITIONS.get(newState.wristState));

        currentState = newState.wristState;

        System.out.println("Wrist: " + newState.toString() + " Angle: " +
                Constants.Subsystems.Wrist.POSITIONS.get(newState.wristState));
    }

    void setTargetAngle(double angle) {
        double rotationsNeeded = angle / 360.0 / Constants.Subsystems.Wrist.MOTOR_REDUCTION; // was a negative on angle

        motor.getPIDController().setReference(rotationsNeeded,
                ControlType.kPosition);
    }

    public boolean isAt(ArmState state) {
        return isAt(state.wristState);
    }

    public boolean isAt(WristState state) {
        boolean there = Math.abs(getAngle() -
                Constants.Subsystems.Wrist.POSITIONS.get(
                        state)) <= Constants.Subsystems.Wrist.TARGET_POSITION_TOLERANCE;

        return there;
    }

    boolean wristDirectDriveEnabled = false;

    public void enableDirectDrive() {
        if (!isDoneMoving)
            return;

        wristDirectDriveEnabled = true;
    }

    public void disableDirectDrive() {
        wristDirectDriveEnabled = false;

        if (!isDoneMoving)
            return;

        setTargetAngle(getAngle());
    }

    public void directDrive(double power) {
        if (!isDoneMoving || !wristDirectDriveEnabled)
            return;

        motor.set(power);
    }

    public boolean crashDetected() {
        return false;
    }
}