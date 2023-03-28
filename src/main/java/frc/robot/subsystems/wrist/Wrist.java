package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
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

        motor.getPIDController().setOutputRange(Constants.Subsystems.Wrist.MIN_SPEED, Constants.Subsystems.Wrist.MAX_SPEED);//was 0.5
        motor.getPIDController().setP(Constants.Subsystems.Wrist.PIDF.P);
        motor.getPIDController().setI(Constants.Subsystems.Wrist.PIDF.I);
        motor.getPIDController().setD(Constants.Subsystems.Wrist.PIDF.D);
        motor.getPIDController().setFF(Constants.Subsystems.Wrist.PIDF.F);

        motor.setClosedLoopRampRate(Constants.Subsystems.Wrist.RAMP_RATE);

        motor.setSmartCurrentLimit(Constants.Subsystems.Wrist.CURRENT_LIMIT);
        motor.setIdleMode(IdleMode.kBrake);

        motor.setSoftLimit(SoftLimitDirection.kForward, Constants.Subsystems.Wrist.MAX_POSITION);
        motor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Subsystems.Wrist.MIN_POSITION);

        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        encoder.setPositionConversionFactor(360);

        correctNeo();

        setState(Constants.Subsystems.Arm.INITIAL_STATE);
    }

    @Override
    public void periodic() {
        if (currentState != null)
            isDoneMoving = isAt(currentState);


    }

    double getAbsoluteEncoderAngle() {
        return -encoder.getPosition() + 180;
    }

    public double getAngle() {
        return motor2Degrees(motor.getEncoder().getPosition());
    }

    double degrees2Motor(double x) {
        return (x / 360.0) / Constants.Subsystems.Wrist.MOTOR_REDUCTION;
    }

    double motor2Degrees(double x) {
        return x * Constants.Subsystems.Wrist.MOTOR_REDUCTION * 360.0;
    }

    void correctNeo() {
        motor.getEncoder().setPosition(degrees2Motor(getAbsoluteEncoderAngle()));
    }

    public void setState(ArmState newState) {
        setTargetAngle(
                Constants.Subsystems.Wrist.POSITIONS.get(newState.wristState));

        currentState = newState.wristState;

        System.out.println("Wrist: " + newState.toString() + " Angle: " +
                Constants.Subsystems.Wrist.POSITIONS.get(newState.wristState));
    }

    void setTargetAngle(double angle) {
        double rotationsNeeded = degrees2Motor(angle); // wasa negative on angle

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