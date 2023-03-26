package frc.robot.subsystems.forearm;

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

public class Forearm extends SubsystemBase implements IStateControllable<ArmState> {
    private CANSparkMax forearmMotor;

    SparkMaxAbsoluteEncoder forearmEncoder;

    ForearmState currentForearmState;

    boolean isDoneMoving = true;

    public Forearm() {
        forearmMotor = new CANSparkMax(
                Constants.Subsystems.Forearm.MOTOR_CHANNEL, MotorType.kBrushless);

        forearmMotor.getPIDController().setOutputRange(-0.5, 0.75);
        forearmMotor.getPIDController().setP(Constants.Subsystems.Forearm.PIDF.P);
        forearmMotor.getPIDController().setI(Constants.Subsystems.Forearm.PIDF.I);
        forearmMotor.getPIDController().setD(Constants.Subsystems.Forearm.PIDF.D);
        forearmMotor.getPIDController().setFF(Constants.Subsystems.Forearm.PIDF.F);

        forearmMotor.setClosedLoopRampRate(0.5);

        forearmMotor.setSmartCurrentLimit(10);
        forearmMotor.setIdleMode(IdleMode.kBrake);

        forearmEncoder = forearmMotor.getAbsoluteEncoder(Type.kDutyCycle);
        forearmEncoder.setPositionConversionFactor(360);

        correctForearmNeo();
    }

    @Override
    public void periodic() {
        if (currentForearmState != null)
            isDoneMoving = isAt(currentForearmState);
    }

    double getAbsoluteEncoderForearmAngle() {
        return (forearmEncoder.getPosition() - 180) * Constants.Subsystems.Forearm.ENCODER_REDUCTION;
    }

    double getForearmAngle() {
        return -forearmMotor.getEncoder().getPosition() *
                Constants.Subsystems.Forearm.MOTOR_REDUCTION * 360.0;
    }

    void correctForearmNeo() {
        forearmMotor.getEncoder().setPosition(
                (-getAbsoluteEncoderForearmAngle() / 360.0) /
                        Constants.Subsystems.Forearm.MOTOR_REDUCTION);
    }

    public void setState(ArmState newState) {
        setForearmTargetAngle(
                Constants.Subsystems.Forearm.POSITIONS.get(newState.forearmState));

        currentForearmState = newState.forearmState;

        System.out.println("Forearm: " + newState.toString() + " Angle: " +
                Constants.Subsystems.Forearm.POSITIONS.get(newState.forearmState));
    }

    void setForearmTargetAngle(double angle) {
        double rotationsNeeded = -angle / 360.0 / Constants.Subsystems.Forearm.MOTOR_REDUCTION;

        forearmMotor.getPIDController().setReference(rotationsNeeded,
                ControlType.kPosition);
    }

    public boolean isAt(ArmState state) {
        return isAt(state.forearmState);
    }

    public boolean isAt(ForearmState state) {
        boolean there = Math.abs(getForearmAngle() -
                Constants.Subsystems.Forearm.POSITIONS.get(
                        state)) <= Constants.Subsystems.Forearm.TARGET_POSITION_TOLERANCE;

        return there;
    }

    boolean forearmDirectDriveEnabled = false;

    public void enableForearmDirectDrive() {
        if (!isDoneMoving)
            return;

        forearmDirectDriveEnabled = true;
    }

    public void disableForearmDirectDrive() {
        forearmDirectDriveEnabled = false;

        if (!isDoneMoving)
            return;

        setForearmTargetAngle(getForearmAngle());
    }

    public void directDriveForearm(double power) {
        if (!isDoneMoving || !forearmDirectDriveEnabled)
            return;

        forearmMotor.set(power);
    }

    public boolean crashDetected() {
        return false;
    }
}