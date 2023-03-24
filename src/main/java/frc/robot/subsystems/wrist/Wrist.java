package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.util.statemachine.IStateControllable;

public class Wrist extends SubsystemBase implements IStateControllable<ArmState> {
    private CANSparkMax wristMotor;

    AnalogPotentiometer wristPotentiometer;

    WristState currentWristState;

    boolean isDoneMoving = true;

    public Wrist() {
        wristMotor = new CANSparkMax(
                Constants.Subsystems.Forearm.MOTOR_CHANNEL, MotorType.kBrushless);

        wristMotor.getPIDController().setOutputRange(-0.5, 0.75);
        wristMotor.getPIDController().setP(Constants.Subsystems.Forearm.PIDF.P);
        wristMotor.getPIDController().setI(Constants.Subsystems.Forearm.PIDF.I);
        wristMotor.getPIDController().setD(Constants.Subsystems.Forearm.PIDF.D);
        wristMotor.getPIDController().setFF(Constants.Subsystems.Forearm.PIDF.F);

        wristMotor.setClosedLoopRampRate(0.5);

        wristMotor.setSmartCurrentLimit(10);
        wristMotor.setIdleMode(IdleMode.kBrake);

        wristPotentiometer = new AnalogPotentiometer(Constants.Subsystems.Wrist.ENCODER_CHANNEL, 360);

        correctForearmNeo();
    }

    @Override
    public void periodic() {
        if (currentWristState != null)
            isDoneMoving = isAt(currentWristState);
    }

    double getAbsoluteEncoderForearmAngle() { // should only be used for correcting
        // return ((forearmPotentiometer.get() + 210) % 360) - 180;// -
        // getShoulderPosition();
        return 0;
    }

    double getForearmAngle() {
        return -wristMotor.getEncoder().getPosition() *
                Constants.Subsystems.Wrist.MOTOR_REDUCTION * 360.0;
    }

    void correctForearmNeo() {
        wristMotor.getEncoder().setPosition(
                (-getAbsoluteEncoderForearmAngle() / 360.0) /
                        Constants.Subsystems.Wrist.MOTOR_REDUCTION);
    }

    public void setState(ArmState newState) {
        setForearmTargetAngle(
                Constants.Subsystems.Wrist.POSITIONS.get(newState.wristState));

        currentWristState = newState.wristState;

        System.out.println("Forearm: " + newState.toString() + " Angle: " +
                Constants.Subsystems.Wrist.POSITIONS.get(newState.wristState));
    }

    void setForearmTargetAngle(double angle) {
        double rotationsNeeded = -angle / 360.0 / Constants.Subsystems.Wrist.MOTOR_REDUCTION;

        wristMotor.getPIDController().setReference(rotationsNeeded,
                ControlType.kPosition);
    }

    public boolean isAt(ArmState state) {
        return isAt(state.wristState);
    }

    public boolean isAt(WristState state) {
        boolean there = Math.abs(getForearmAngle() -
                Constants.Subsystems.Wrist.POSITIONS.get(
                        state)) <= Constants.Subsystems.Wrist.TARGET_POSITION_TOLERANCE;

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

        wristMotor.set(power);
    }

    public boolean crashDetected() {
        return false;
    }
}