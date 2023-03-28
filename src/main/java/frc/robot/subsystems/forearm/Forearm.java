package frc.robot.subsystems.forearm;

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

public class Forearm extends SubsystemBase implements IStateControllable<ArmState> {
    private CANSparkMax motor;

    SparkMaxAbsoluteEncoder encoder;

    ForearmState currentState;

    boolean isDoneMoving = true;

    public Forearm() {
        motor = new CANSparkMax(
                Constants.Subsystems.Forearm.MOTOR_CHANNEL, MotorType.kBrushless);

        motor.getPIDController().setOutputRange(Constants.Subsystems.Forearm.MIN_SPEED,
                Constants.Subsystems.Forearm.MAX_SPEED);// was 25
        motor.getPIDController().setP(Constants.Subsystems.Forearm.PIDF.P);
        motor.getPIDController().setI(Constants.Subsystems.Forearm.PIDF.I);
        motor.getPIDController().setD(Constants.Subsystems.Forearm.PIDF.D);
        motor.getPIDController().setFF(Constants.Subsystems.Forearm.PIDF.F);

        motor.setClosedLoopRampRate(Constants.Subsystems.Forearm.RAMP_RATE);

        motor.setSmartCurrentLimit(Constants.Subsystems.Forearm.CURRENT_LIMIT);
        motor.setIdleMode(IdleMode.kBrake);

        motor.setSoftLimit(SoftLimitDirection.kForward, Constants.Subsystems.Forearm.MAX_POSITION);
        motor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Subsystems.Forearm.MIN_POSITION);

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

        // System.out.println(motor.getEncoder().getPosition());
    }

    double getAbsoluteEncoderAngle() {
        return encoder.getPosition() - 180;
    }

    public double getAngle() {
        return motor2Degrees(motor.getEncoder().getPosition());
    }

    double degrees2Motor(double x) {
        return (-x / 360.0) / Constants.Subsystems.Forearm.MOTOR_REDUCTION;
    }

    double motor2Degrees(double x) {
        return -x * Constants.Subsystems.Forearm.MOTOR_REDUCTION * 360.0;
    }

    void correctNeo() {
        motor.getEncoder().setPosition(degrees2Motor(getAbsoluteEncoderAngle()));
    }

    public void setState(ArmState newState) {
        setTargetAngle(
                Constants.Subsystems.Forearm.POSITIONS.get(newState.forearmState));

        currentState = newState.forearmState;

        System.out.println("Forearm: " + newState.toString() + " Angle: " +
                Constants.Subsystems.Forearm.POSITIONS.get(newState.forearmState));
    }

    void setTargetAngle(double angle) {
        double rotationsNeeded = degrees2Motor(angle);

        motor.getPIDController().setReference(rotationsNeeded,
                ControlType.kPosition);
    }

    public boolean isAt(ArmState state) {
        return isAt(state.forearmState);
    }

    public boolean isAt(ForearmState state) {
        boolean there = Math.abs(getAngle() -
                Constants.Subsystems.Forearm.POSITIONS.get(
                        state)) <= Constants.Subsystems.Forearm.TARGET_POSITION_TOLERANCE;

        return there;
    }

    boolean forearmDirectDriveEnabled = false;

    public void enableDirectDrive() {
        if (!isDoneMoving)
            return;

        forearmDirectDriveEnabled = true;
    }

    public void disableDirectDrive() {
        forearmDirectDriveEnabled = false;

        if (!isDoneMoving)
            return;

        setTargetAngle(getAngle());
    }

    public void directDrive(double power) {
        if (!isDoneMoving || !forearmDirectDriveEnabled)
            return;

        motor.set(power);
    }

    public boolean crashDetected() {
        return false;
    }
}