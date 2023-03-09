package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.statemachine.StateMachine;
import frc.robot.util.statemachine.TransitionGraph;
import frc.robot.util.statemachine.TransitionGraphNode;

public class Arm extends SubsystemBase {
    private DoubleSolenoid topPiston;
    private DoubleSolenoid bottomPiston;

    AnalogPotentiometer shoulderPotentiometer;

    private DoubleSolenoid hopperPiston;

    private CANSparkMax forearmMotor;

    AnalogPotentiometer forearmPotentiometer;

    StateMachine<ArmPosition> armStateMachine;
    ShoulderState currentShoulderState;
    ForearmState currentForearmState;

    ADXL345_I2C forearmIMU;

    ArmPosition queuedArmPosition = null;

    public Arm() {
        topPiston = new DoubleSolenoid(
                Constants.Subsystems.Pneumatics.BASE_PCM, PneumaticsModuleType.REVPH,
                Constants.Subsystems.Arm.TOP_PISTON_SOLENOID_CHANNEL,
                Constants.Subsystems.Arm.TOP_PISTON_SOLENOID_CHANNEL + 1);

        bottomPiston = new DoubleSolenoid(
                Constants.Subsystems.Pneumatics.BASE_PCM, PneumaticsModuleType.REVPH,
                Constants.Subsystems.Arm.BOTTOM_PISTON_SOLENOID_CHANNEL,
                Constants.Subsystems.Arm.BOTTOM_PISTON_SOLENOID_CHANNEL + 1);

        hopperPiston = new DoubleSolenoid(
                Constants.Subsystems.Pneumatics.BASE_PCM, PneumaticsModuleType.REVPH,
                Constants.Subsystems.Arm.HOPPER_SOLENOID_CHANNEL,
                Constants.Subsystems.Arm.HOPPER_SOLENOID_CHANNEL + 1);

        forearmMotor = new CANSparkMax(
                Constants.Subsystems.Arm.FOREARM_MOTOR_CHANNEL, MotorType.kBrushless);

        forearmMotor.getPIDController().setOutputRange(-0.5, 0.5);
        forearmMotor.getPIDController().setP(Constants.Subsystems.Arm.PIDF.P);
        forearmMotor.getPIDController().setI(Constants.Subsystems.Arm.PIDF.I);
        forearmMotor.getPIDController().setD(Constants.Subsystems.Arm.PIDF.D);
        forearmMotor.getPIDController().setFF(Constants.Subsystems.Arm.PIDF.F);

        forearmMotor.setClosedLoopRampRate(0.5);

        forearmMotor.setSmartCurrentLimit(10);
        forearmMotor.setIdleMode(IdleMode.kBrake);

        // forearmPotentiometer = new
        // AnalogPotentiometer(Constants.Subsystems.Arm.FOREARM_ENCODER_CHANNEL,
        // 360);
        forearmIMU = new ADXL345_I2C(Port.kOnboard, Range.k16G);

        shoulderPotentiometer = new AnalogPotentiometer(
                Constants.Subsystems.Arm.SHOULDER_POTENTIOMETER_CHANNEL, 1);

        armStateMachine = new StateMachine<>(ArmPosition.Store);
        setShoulderState(ShoulderState.Base2);
        setForearmState(ForearmState.Store);
        setHopperState(HopperState.Retracted);
        // BE CAREFUL HERE, THESE CALLS SYNC UP EVERYTHING!!!

        populateLowerHopperGrabGraph();
        populateUpperHopperGrabGraph();
        populateGroundIntakeGraph();
        populateDoubleSubstationGraph();
        populateHybridGraph();
        populateStoreGraph();
        populateBase4Cone2Graph();
        populateBase4Cube2Graph();
        populateBase4Cube1Graph();
        populateBase2Cone1Graph();
        populateBase1Cube1Graph();

        correctForearmNeo();
    }

    @Override
    public void periodic() {
        armStateMachine.update();
    }

    public void setArmState(ArmPosition newState) {
        System.out.println("Arm to: " + newState);

        armStateMachine.transitionTo(newState);
    }

    public boolean isDoneMoving() {
        return isDoneMoving;
    }

    boolean inCancelMode = false;

    public void startCancelMode() {
        armStateMachine.cancelActiveTransition();

        setForearmTargetAngle(getForearmAngle());

        inCancelMode = true;

        System.out.println("!!! cancelled !!!");
    }

    public void endCancelMode() {
        armStateMachine.forceState(ArmPosition.Store);

        setShoulderState(ShoulderState.Base2);
        setForearmState(ForearmState.Store);
        setHopperState(HopperState.Retracted);

        inCancelMode = false;
    }

    public void declareInStore() {
        forearmMotor.getEncoder().setPosition(
                (-Constants.Subsystems.Arm.FOREARM_POSITION.get(ForearmState.Store) /
                        360.0) /
                        Constants.Subsystems.Arm.FOREARM_MOTOR_REDUCTION);

        armStateMachine.transitionTo(ArmPosition.Store);
    }

    // region shoulder hardware interfacing

    double getShoulderAngle() { // Striaght up is zero, positive is towards the front
        switch (currentShoulderState) { // TODO: VERY SKETCHY!!! PLEASE REPLACE WITH
                                        // SENSOR FEEDBACK
            case Base1:
                return 51.1;
            case Base2:
                return 25.2;
            case Base3:
                return 11.4;
            case Base4:
                return -22.3;
        }

        return 25.2;
    }

    void setTopPiston(Value value) {
        topPiston.set((value == Value.kForward) ? Value.kForward : Value.kReverse);
    }

    void setBottomPiston(Value value) {
        bottomPiston.set((value == Value.kReverse) ? Value.kForward
                : Value.kReverse);
    }

    long lastShoulderActuationTime = System.currentTimeMillis();

    public void setShoulderState(ShoulderState newState) {
        switch (newState) {
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

        currentShoulderState = newState;

        lastShoulderActuationTime = System.currentTimeMillis();

        System.out.println("Shoulder: " + newState.toString());
    }

    public boolean isAtShoulderState(ShoulderState state) {
        return Math.abs(shoulderPotentiometer.get() -
                Constants.Subsystems.Arm.SHOULDER_POSITIONS
                        .get(state)) <= Constants.Subsystems.Arm.SHOULDER_TARGET_TOLERANCE;
        // return (System.currentTimeMillis() - lastShoulderActuationTime) >= 1500;
        // TODO: check sensors
    }

    // endregion

    // region forearm hardware interfacing

    double getAbsoluteEncoderForearmAngle() { // should only be used for correcting
        // return ((forearmPotentiometer.get() + 210) % 360) - 180;// -
        // getShoulderPosition();
        return -Math.atan(forearmIMU.getZ() / forearmIMU.getY()) * (180 / Math.PI);
    }

    double getForearmAngle() {
        return -forearmMotor.getEncoder().getPosition() *
                Constants.Subsystems.Arm.FOREARM_MOTOR_REDUCTION * 360.0;
    }

    void correctForearmNeo() {
        forearmMotor.getEncoder().setPosition(
                (-getAbsoluteEncoderForearmAngle() / 360.0) /
                        Constants.Subsystems.Arm.FOREARM_MOTOR_REDUCTION);
    }

    void setForearmState(ForearmState newState) {
        setForearmTargetAngle(
                Constants.Subsystems.Arm.FOREARM_POSITION.get(newState));

        currentForearmState = newState;

        System.out.println("Forearm: " + newState.toString() + " Angle: " +
                Constants.Subsystems.Arm.FOREARM_POSITION.get(newState));
    }

    void setForearmTargetAngle(double angle) {
        double rotationsNeeded = -angle / 360.0 / Constants.Subsystems.Arm.FOREARM_MOTOR_REDUCTION;

        forearmMotor.getPIDController().setReference(rotationsNeeded,
                ControlType.kPosition);
    }

    boolean isAtForearmState(ForearmState state) {
        boolean there = Math.abs(getForearmAngle() -
                Constants.Subsystems.Arm.FOREARM_POSITION.get(
                        currentForearmState)) <= Constants.Subsystems.Arm.FOREARM_TARGET_POSITION_TOLERANCE;

        if (there)
            System.out.println("Forearm Hit: " + getForearmAngle());

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
        if (inCancelMode) {
            forearmMotor.set(power);

            return;
        }

        if (!isDoneMoving || !forearmDirectDriveEnabled)
            return;

        forearmMotor.set(power);
    }

    public boolean isInCancelMode() {
        return inCancelMode;
    }

    // endregion

    // region hopper hardware interfacing

    void setHopperPiston(Value value) {
        hopperPiston.set((value == Value.kForward) ? Value.kForward
                : Value.kReverse);
    }

    long lastHopperActuationTime = System.currentTimeMillis();

    public void setHopperState(HopperState newState) {
        hopperPiston.set((newState == HopperState.Extended) ? Value.kForward
                : Value.kReverse);

        lastHopperActuationTime = System.currentTimeMillis();

        System.out.println("Hopper: " + newState.toString());
    }

    boolean isAtHopperState(HopperState state) {
        return (System.currentTimeMillis() - lastHopperActuationTime) >= 500; // we don't actually have a sensor
        // for
        // this, might as well have a way
        // just in
        // case though
    }

    // endregion

    // region Helper Functions for building the State Machine

    boolean isDoneMoving = true;

    TransitionGraphNode begin(TransitionGraphNode next) {
        return new TransitionGraphNode(
                () -> isDoneMoving = false, () -> true, next);
    }

    TransitionGraphNode shoulderMovement(ShoulderState newState,
            TransitionGraphNode next) {
        return new TransitionGraphNode(() -> setShoulderState(newState),
                () -> isAtShoulderState(newState), next);
    }

    TransitionGraphNode forearmMovement(ForearmState newState,
            TransitionGraphNode next) {
        return new TransitionGraphNode(() -> setForearmState(newState),
                () -> isAtForearmState(newState), next);
    }

    TransitionGraphNode hopperMovement(HopperState newState,
            TransitionGraphNode next) {
        return new TransitionGraphNode(
                () -> setHopperState(newState), () -> isAtHopperState(newState), next);
    }

    TransitionGraphNode parallelMovement(ForearmState newForearmState,
            ShoulderState newShoulderState,
            TransitionGraphNode next) {
        return new TransitionGraphNode(
                () -> {
                    setForearmState(newForearmState);
                    setShoulderState(newShoulderState);
                },
                () -> isAtForearmState(newForearmState) &&
                        isAtShoulderState(newShoulderState),
                next);
    }

    TransitionGraphNode parallelMovement(ForearmState newForearmState,
            HopperState newHopperState,
            TransitionGraphNode next) {
        return new TransitionGraphNode(
                () -> {
                    setForearmState(newForearmState);
                    setHopperState(newHopperState);
                },
                () -> isAtForearmState(newForearmState) &&
                        isAtHopperState(newHopperState),
                next);
    }

    TransitionGraphNode parallelMovement(ShoulderState newShoulderState,
            HopperState newHopperState,
            TransitionGraphNode next) {
        return new TransitionGraphNode(
                () -> {
                    setShoulderState(newShoulderState);
                    setHopperState(newHopperState);
                },
                () -> isAtShoulderState(newShoulderState) &&
                        isAtHopperState(newHopperState),
                next);
    }

    TransitionGraphNode parallelMovement(ForearmState newForearmState,
            ShoulderState newShoulderState,
            HopperState newHopperState,
            TransitionGraphNode next) {
        return new TransitionGraphNode(
                () -> {
                    setForearmState(newForearmState);
                    setShoulderState(newShoulderState);
                    setHopperState(newHopperState);
                },
                () -> isAtForearmState(newForearmState) &&
                        isAtShoulderState(newShoulderState) &&
                        isAtHopperState(newHopperState),
                next);
    }

    TransitionGraphNode done() {
        return new TransitionGraphNode(() -> {
            isDoneMoving = true;
            System.out.println("--------------------");
        }, () -> true, null);
    }

    // endregion

    void populateLowerHopperGrabGraph() {
        armStateMachine.addBehaviour(ArmPosition.LowerHopperGrab, () -> {
        });
        armStateMachine.addTransitionGraph(
                null, ArmPosition.LowerHopperGrab,
                new TransitionGraph(begin(
                        // forearmMovement(ForearmState.HopperGrab,
                        new TransitionGraphNode(
                                () -> currentShoulderState == ShoulderState.Base4,
                                new TransitionGraphNode(
                                        () -> currentForearmState == ForearmState.DoubleSubstation,
                                        hopperMovement(
                                                HopperState.Extended,
                                                shoulderMovement(
                                                        ShoulderState.Base1,
                                                        forearmMovement(
                                                                ForearmState.UpperHopperGrab,
                                                                hopperMovement(HopperState.Retracted,
                                                                        forearmMovement(
                                                                                ForearmState.LowerHopperGrab,
                                                                                done()))))),
                                        forearmMovement(
                                                ForearmState.DoubleSubstation,
                                                hopperMovement(
                                                        HopperState.Extended,
                                                        forearmMovement(
                                                                ForearmState.UpperHopperGrab,
                                                                shoulderMovement(
                                                                        ShoulderState.Base1,
                                                                        forearmMovement(
                                                                                ForearmState.LowerHopperGrab,
                                                                                hopperMovement(HopperState.Retracted,
                                                                                        done()))))))),
                                new TransitionGraphNode(
                                        () -> currentShoulderState == ShoulderState.Base2,
                                        parallelMovement(
                                                ForearmState.UpperHopperGrab, ShoulderState.Base1,
                                                        forearmMovement(ForearmState.LowerHopperGrab,
                                                                done())),
                                        new TransitionGraphNode(
                                                () -> currentShoulderState == ShoulderState.Base1,
                                                new TransitionGraphNode(
                                                        () -> (currentForearmState == ForearmState.LowerHopperGrab) ||
                                                                (currentForearmState == ForearmState.UpperHopperGrab),
                                                        forearmMovement(ForearmState.LowerHopperGrab,
                                                                done()),
                                                        forearmMovement(
                                                                ForearmState.Intermediate,
                                                                shoulderMovement(
                                                                        ShoulderState.Base2,
                                                                        parallelMovement(
                                                                                ForearmState.UpperHopperGrab, ShoulderState.Base1,
                                                                                        forearmMovement(
                                                                                                ForearmState.LowerHopperGrab,
                                                                                                done()))))),
                                                done()))))));
    }

    void populateUpperHopperGrabGraph() {
        armStateMachine.addBehaviour(ArmPosition.UpperHopperGrab, () -> {
        });
        armStateMachine.addTransitionGraph(
                null, ArmPosition.UpperHopperGrab,
                new TransitionGraph(begin(
                        // forearmMovement(ForearmState.HopperGrab,
                        new TransitionGraphNode(
                                () -> currentShoulderState == ShoulderState.Base4,
                                new TransitionGraphNode(
                                        () -> currentForearmState == ForearmState.DoubleSubstation,
                                        hopperMovement(HopperState.Extended,
                                                shoulderMovement(
                                                        ShoulderState.Base1,
                                                        forearmMovement(
                                                                ForearmState.UpperHopperGrab,
                                                                hopperMovement(HopperState.Retracted,
                                                                        done())))),
                                        forearmMovement(
                                                ForearmState.DoubleSubstation,
                                                hopperMovement(
                                                        HopperState.Extended,
                                                        shoulderMovement(
                                                                ShoulderState.Base1,
                                                                forearmMovement(
                                                                        ForearmState.UpperHopperGrab,
                                                                        hopperMovement(HopperState.Retracted,
                                                                                done())))))),
                                new TransitionGraphNode(
                                        () -> currentShoulderState == ShoulderState.Base2,
                                        parallelMovement(
                                                ForearmState.UpperHopperGrab, ShoulderState.Base1, done()),
                                        new TransitionGraphNode(
                                                () -> currentShoulderState == ShoulderState.Base1,
                                                new TransitionGraphNode(
                                                        () -> (currentForearmState == ForearmState.LowerHopperGrab) ||
                                                                (currentForearmState == ForearmState.UpperHopperGrab),
                                                        forearmMovement(ForearmState.UpperHopperGrab,
                                                                done()),
                                                                parallelMovement(
                                                                    ForearmState.Intermediate, ShoulderState.Base2,
                                                                        forearmMovement(
                                                                                ForearmState.UpperHopperGrab,
                                                                                shoulderMovement(ShoulderState.Base1,
                                                                                        done())))),
                                                done()))))));
    }

    void populateGroundIntakeGraph() {
        armStateMachine.addBehaviour(ArmPosition.GroundIntake, () -> {
        });
        armStateMachine.addTransitionGraph(
                null, ArmPosition.GroundIntake,
                new TransitionGraph(begin(
                        // forearmMovement(ForearmState.GroundIntake,
                        new TransitionGraphNode(
                                () -> currentShoulderState == ShoulderState.Base4,
                                forearmMovement(
                                        ForearmState.DoubleSubstation,
                                        hopperMovement(
                                                HopperState.Extended,
                                                shoulderMovement(
                                                        ShoulderState.Base2,
                                                        forearmMovement(
                                                                ForearmState.Store,
                                                                hopperMovement(
                                                                        HopperState.Retracted,
                                                                        forearmMovement(
                                                                                ForearmState.Intermediate,
                                                                                shoulderMovement(
                                                                                        ShoulderState.Base1,
                                                                                        forearmMovement(
                                                                                                ForearmState.GroundIntake,
                                                                                                done())))))))),
                                new TransitionGraphNode(
                                        () -> currentShoulderState == ShoulderState.Base2,
                                        hopperMovement(
                                                HopperState.Retracted,
                                                forearmMovement(
                                                        ForearmState.Intermediate,
                                                        shoulderMovement(
                                                                ShoulderState.Base1,
                                                                forearmMovement(ForearmState.GroundIntake,
                                                                        done())))),
                                        new TransitionGraphNode(
                                                () -> currentShoulderState == ShoulderState.Base1,
                                                new TransitionGraphNode(
                                                        () -> (currentForearmState == ForearmState.LowerHopperGrab) ||
                                                                (currentForearmState == ForearmState.UpperHopperGrab),
                                                        hopperMovement(
                                                                HopperState.Retracted,
                                                                shoulderMovement(
                                                                        ShoulderState.Base2,
                                                                        forearmMovement(
                                                                                ForearmState.Intermediate,
                                                                                shoulderMovement(
                                                                                        ShoulderState.Base1,
                                                                                        forearmMovement(
                                                                                                ForearmState.GroundIntake,
                                                                                                done()))))),
                                                        forearmMovement(ForearmState.GroundIntake, done())),
                                                done()))))));
    }

    void populateDoubleSubstationGraph() {
        armStateMachine.addBehaviour(ArmPosition.DoubleSubstation, () -> {
        });
        armStateMachine.addTransitionGraph(
                null, ArmPosition.DoubleSubstation,
                new TransitionGraph(begin(
                        // forearmMovement(ForearmState.DoubleSubstation,
                        new TransitionGraphNode(
                                () -> currentShoulderState == ShoulderState.Base4,
                                forearmMovement(ForearmState.DoubleSubstation, done()),
                                new TransitionGraphNode(
                                        () -> currentShoulderState == ShoulderState.Base2,
                                        hopperMovement(
                                                HopperState.Extended,
                                                forearmMovement(
                                                        ForearmState.Store,
                                                        forearmMovement(ForearmState.DoubleSubstation,
                                                                shoulderMovement(
                                                                        ShoulderState.Base4, done())))),
                                        new TransitionGraphNode(
                                                () -> currentShoulderState == ShoulderState.Base1,
                                                new TransitionGraphNode(
                                                        () -> (currentForearmState == ForearmState.LowerHopperGrab) ||
                                                                (currentForearmState == ForearmState.UpperHopperGrab),

                                                        hopperMovement(
                                                                HopperState.Extended,
                                                                parallelMovement(
                                                                        ForearmState.Store, ShoulderState.Base2,
                                                                        forearmMovement(
                                                                                ForearmState.DoubleSubstation,
                                                                                shoulderMovement(ShoulderState.Base4,
                                                                                        done())))),
                                                        forearmMovement(
                                                                ForearmState.Intermediate,
                                                                shoulderMovement(
                                                                        ShoulderState.Base2,
                                                                        hopperMovement(
                                                                                HopperState.Extended,
                                                                                forearmMovement(
                                                                                        ForearmState.Store,
                                                                                        forearmMovement(
                                                                                                ForearmState.DoubleSubstation,
                                                                                                shoulderMovement(
                                                                                                        ShoulderState.Base4,
                                                                                                        done()))))))),
                                                done()))))));
    }

    void populateHybridGraph() {
        armStateMachine.addBehaviour(ArmPosition.Hybrid, () -> {
        });
        armStateMachine.addTransitionGraph(
                null, ArmPosition.Hybrid,
                new TransitionGraph(begin(
                        // forearmMovement(ForearmState.Base1Hybrid,
                        new TransitionGraphNode(
                                () -> currentShoulderState == ShoulderState.Base4,
                                hopperMovement(
                                        HopperState.Extended,
                                        forearmMovement(
                                                ForearmState.DoubleSubstation,
                                                shoulderMovement(
                                                        ShoulderState.Base2,
                                                        forearmMovement(
                                                                ForearmState.Store,
                                                                hopperMovement(
                                                                        HopperState.Retracted,
                                                                        forearmMovement(
                                                                                ForearmState.Intermediate,
                                                                                shoulderMovement(
                                                                                        ShoulderState.Base1,
                                                                                        forearmMovement(
                                                                                                ForearmState.Base1Hybrid,
                                                                                                done())))))))),
                                new TransitionGraphNode(
                                        () -> currentShoulderState == ShoulderState.Base2,
                                        forearmMovement(
                                                ForearmState.Intermediate,
                                                shoulderMovement(
                                                        ShoulderState.Base1,
                                                        forearmMovement(ForearmState.Base1Hybrid, done()))),
                                        new TransitionGraphNode(
                                                () -> currentShoulderState == ShoulderState.Base1,
                                                new TransitionGraphNode(
                                                        () -> (currentForearmState == ForearmState.LowerHopperGrab) ||
                                                                (currentForearmState == ForearmState.UpperHopperGrab),
                                                        shoulderMovement(
                                                                ShoulderState.Base2,
                                                                forearmMovement(
                                                                        ForearmState.Intermediate,
                                                                        shoulderMovement(
                                                                                ShoulderState.Base1,
                                                                                forearmMovement(
                                                                                        ForearmState.Base1Hybrid,
                                                                                        done())))),
                                                        forearmMovement(ForearmState.Base1Hybrid, done())),
                                                done()))))));
    }

    void populateStoreGraph() {
        armStateMachine.addBehaviour(ArmPosition.Store, () -> {
        });
        armStateMachine.addTransitionGraph(
                null, ArmPosition.Store,
                new TransitionGraph(begin(
                        // forearmMovement(ForearmState.Store,
                        new TransitionGraphNode(
                                () -> currentShoulderState == ShoulderState.Base4,
                                new TransitionGraphNode(
                                        () -> currentForearmState == ForearmState.DoubleSubstation,
                                        hopperMovement(
                                                HopperState.Extended,
                                                shoulderMovement(
                                                        ShoulderState.Base1,
                                                        forearmMovement(
                                                                ForearmState.LowerHopperGrab,
                                                                hopperMovement(
                                                                        HopperState.Retracted,
                                                                        shoulderMovement(
                                                                                ShoulderState.Base2,
                                                                                forearmMovement(ForearmState.Store,
                                                                                        done())))))),
                                        hopperMovement(
                                                HopperState.Extended,
                                                forearmMovement(
                                                        ForearmState.DoubleSubstation,
                                                        shoulderMovement(
                                                                ShoulderState.Base2,
                                                                forearmMovement(
                                                                        ForearmState.Store,
                                                                        hopperMovement(HopperState.Retracted,
                                                                                done())))))),
                                new TransitionGraphNode(
                                        () -> currentShoulderState == ShoulderState.Base2,
                                        forearmMovement(ForearmState.Store, done()),
                                        new TransitionGraphNode(
                                                () -> currentShoulderState == ShoulderState.Base1,
                                                new TransitionGraphNode(
                                                        () -> (currentForearmState == ForearmState.LowerHopperGrab) ||
                                                                (currentForearmState == ForearmState.UpperHopperGrab),
                                                        shoulderMovement(
                                                                ShoulderState.Base2,
                                                                forearmMovement(ForearmState.Store, done())),
                                                        forearmMovement(
                                                                ForearmState.Intermediate,
                                                                shoulderMovement(
                                                                        ShoulderState.Base2,
                                                                        forearmMovement(ForearmState.Store,
                                                                                done())))),
                                                done()))))));
    }

    void populateBase4Cone2Graph() {
        armStateMachine.addBehaviour(ArmPosition.Base4Cone2, () -> {
        });
        armStateMachine.addTransitionGraph(
                null, ArmPosition.Base4Cone2,
                new TransitionGraph(begin(
                        // forearmMovement(ForearmState.Base4Cone2,
                        new TransitionGraphNode(
                                () -> currentShoulderState == ShoulderState.Base4,
                                forearmMovement(ForearmState.Base4Cone2, done()),
                                new TransitionGraphNode(
                                        () -> currentShoulderState == ShoulderState.Base2,
                                        hopperMovement(
                                                HopperState.Extended,
                                                forearmMovement(
                                                        ForearmState.Store,
                                                        forearmMovement(
                                                                ForearmState.DoubleSubstation,
                                                                shoulderMovement(
                                                                        ShoulderState.Base4,
                                                                        forearmMovement(ForearmState.Base4Cone2,
                                                                                done()))))),
                                        new TransitionGraphNode(
                                                () -> currentShoulderState == ShoulderState.Base1,
                                                new TransitionGraphNode(
                                                        () -> (currentForearmState == ForearmState.LowerHopperGrab) ||
                                                                (currentForearmState == ForearmState.UpperHopperGrab),
                                                        hopperMovement(
                                                                HopperState.Extended,
                                                                forearmMovement(
                                                                        ForearmState.DoubleSubstation,
                                                                        shoulderMovement(
                                                                                ShoulderState.Base4,
                                                                                forearmMovement(ForearmState.Base4Cone2,
                                                                                        done())))),
                                                        forearmMovement(
                                                                ForearmState.Intermediate,
                                                                shoulderMovement(
                                                                        ShoulderState.Base2,
                                                                        hopperMovement(
                                                                                HopperState.Extended,
                                                                                forearmMovement(
                                                                                        ForearmState.Store,
                                                                                        forearmMovement(
                                                                                                ForearmState.DoubleSubstation,
                                                                                                shoulderMovement(
                                                                                                        ShoulderState.Base4,
                                                                                                        forearmMovement(
                                                                                                                ForearmState.Base4Cone2,
                                                                                                                done())))))))),
                                                done()))))));
    }

    void populateBase4Cube2Graph() {
        armStateMachine.addBehaviour(ArmPosition.Base4Cube2, () -> {
        });
        armStateMachine.addTransitionGraph(
                null, ArmPosition.Base4Cube2,
                new TransitionGraph(begin(
                        // forearmMovement(ForearmState.Base4Cube2,
                        new TransitionGraphNode(
                                () -> currentShoulderState == ShoulderState.Base4,
                                forearmMovement(ForearmState.Base4Cube2, done()),
                                new TransitionGraphNode(
                                        () -> currentShoulderState == ShoulderState.Base2,
                                        hopperMovement(
                                                HopperState.Extended,
                                                forearmMovement(
                                                        ForearmState.Store,
                                                        forearmMovement(
                                                                ForearmState.DoubleSubstation,
                                                                shoulderMovement(
                                                                        ShoulderState.Base4,
                                                                        forearmMovement(ForearmState.Base4Cube2,
                                                                                done()))))),
                                        new TransitionGraphNode(
                                                () -> currentShoulderState == ShoulderState.Base1,
                                                new TransitionGraphNode(
                                                        () -> (currentForearmState == ForearmState.LowerHopperGrab) ||
                                                                (currentForearmState == ForearmState.UpperHopperGrab),
                                                        hopperMovement(
                                                                HopperState.Extended,
                                                                forearmMovement(
                                                                        ForearmState.DoubleSubstation,
                                                                        shoulderMovement(
                                                                                ShoulderState.Base4,
                                                                                forearmMovement(ForearmState.Base4Cube2,
                                                                                        done())))),
                                                        forearmMovement(
                                                                ForearmState.Intermediate,
                                                                shoulderMovement(
                                                                        ShoulderState.Base2,
                                                                        hopperMovement(
                                                                                HopperState.Extended,
                                                                                forearmMovement(
                                                                                        ForearmState.Store,
                                                                                        forearmMovement(
                                                                                                ForearmState.DoubleSubstation,
                                                                                                shoulderMovement(
                                                                                                        ShoulderState.Base4,
                                                                                                        forearmMovement(
                                                                                                                ForearmState.Base4Cube2,
                                                                                                                done())))))))),
                                                done()))))));
    }

    void populateBase4Cube1Graph() {
        armStateMachine.addBehaviour(ArmPosition.Base4Cube1, () -> {
        });
        armStateMachine.addTransitionGraph(
                null, ArmPosition.Base4Cube1,
                new TransitionGraph(begin(
                        // forearmMovement(ForearmState.Base4Cube1,
                        new TransitionGraphNode(
                                () -> currentShoulderState == ShoulderState.Base4,
                                forearmMovement(ForearmState.Base4Cube1, done()),
                                new TransitionGraphNode(
                                        () -> currentShoulderState == ShoulderState.Base2,
                                        hopperMovement(
                                                HopperState.Extended,
                                                forearmMovement(
                                                        ForearmState.DoubleSubstation,
                                                        shoulderMovement(
                                                                ShoulderState.Base4,
                                                                forearmMovement(ForearmState.Base4Cube1,
                                                                        done())))),
                                        new TransitionGraphNode(
                                                () -> currentShoulderState == ShoulderState.Base1,
                                                new TransitionGraphNode(
                                                        () -> (currentForearmState == ForearmState.LowerHopperGrab) ||
                                                                (currentForearmState == ForearmState.UpperHopperGrab),
                                                        hopperMovement(
                                                                HopperState.Extended,
                                                                forearmMovement(
                                                                        ForearmState.DoubleSubstation,
                                                                        shoulderMovement(
                                                                                ShoulderState.Base4,
                                                                                forearmMovement(ForearmState.Base4Cube1,
                                                                                        done())))),
                                                        forearmMovement(
                                                                ForearmState.Intermediate,
                                                                shoulderMovement(
                                                                        ShoulderState.Base2,
                                                                        hopperMovement(
                                                                                HopperState.Extended,
                                                                                forearmMovement(
                                                                                        ForearmState.Store,
                                                                                        forearmMovement(
                                                                                                ForearmState.DoubleSubstation,
                                                                                                shoulderMovement(
                                                                                                        ShoulderState.Base4,
                                                                                                        forearmMovement(
                                                                                                                ForearmState.Base4Cube1,
                                                                                                                done())))))))),
                                                done()))))));
    }

    void populateBase2Cone1Graph() {
        armStateMachine.addBehaviour(ArmPosition.Base2Cone1, () -> {
        });
        armStateMachine.addTransitionGraph(
                null, ArmPosition.Base2Cone1,
                new TransitionGraph(begin(
                        // forearmMovement(ForearmState.Base2Cone1,
                        new TransitionGraphNode(
                                () -> currentShoulderState == ShoulderState.Base4,
                                hopperMovement(
                                        HopperState.Extended,
                                        forearmMovement(
                                                ForearmState.DoubleSubstation,
                                                shoulderMovement(
                                                        ShoulderState.Base2,
                                                        forearmMovement(
                                                                ForearmState.Store,
                                                                hopperMovement(
                                                                        HopperState.Retracted,
                                                                        forearmMovement(ForearmState.Base2Cone1,
                                                                                done())))))),
                                new TransitionGraphNode(
                                        () -> currentShoulderState == ShoulderState.Base2,
                                        forearmMovement(ForearmState.Base2Cone1, done()),
                                        new TransitionGraphNode(
                                                () -> currentShoulderState == ShoulderState.Base1,
                                                new TransitionGraphNode(
                                                        () -> (currentForearmState == ForearmState.LowerHopperGrab) ||
                                                                (currentForearmState == ForearmState.UpperHopperGrab),
                                                        shoulderMovement(
                                                                ShoulderState.Base2,
                                                                forearmMovement(ForearmState.Base2Cone1,
                                                                        done())),
                                                        forearmMovement(
                                                                ForearmState.Intermediate,
                                                                shoulderMovement(
                                                                        ShoulderState.Base2,
                                                                        forearmMovement(ForearmState.Base2Cone1,
                                                                                done())))),
                                                done()))))));
    }

    void populateBase1Cube1Graph() {
        armStateMachine.addBehaviour(ArmPosition.Base1Cube1, () -> {
        });
        armStateMachine.addTransitionGraph(
                null, ArmPosition.Base1Cube1,
                new TransitionGraph(begin(
                        // forearmMovement(ForearmState.Base1Cube1,
                        new TransitionGraphNode(
                                () -> currentShoulderState == ShoulderState.Base4,
                                hopperMovement(
                                        HopperState.Extended,
                                        forearmMovement(
                                                ForearmState.DoubleSubstation,
                                                shoulderMovement(
                                                        ShoulderState.Base2,
                                                        forearmMovement(
                                                                ForearmState.Store,
                                                                hopperMovement(
                                                                        HopperState.Retracted,
                                                                        forearmMovement(
                                                                                ForearmState.Base1Cube1,
                                                                                shoulderMovement(ShoulderState.Base1,
                                                                                        done()))))))),
                                new TransitionGraphNode(
                                        () -> currentShoulderState == ShoulderState.Base2,
                                        forearmMovement(
                                                ForearmState.Base1Cube1,
                                                shoulderMovement(ShoulderState.Base1, done())),
                                        new TransitionGraphNode(
                                                () -> currentShoulderState == ShoulderState.Base1,
                                                new TransitionGraphNode(
                                                        () -> (currentForearmState == ForearmState.LowerHopperGrab) ||
                                                                (currentForearmState == ForearmState.UpperHopperGrab),
                                                        shoulderMovement(
                                                                ShoulderState.Base2,
                                                                forearmMovement(
                                                                        ForearmState.Base1Cube1,
                                                                        shoulderMovement(ShoulderState.Base1,
                                                                                done()))),
                                                        forearmMovement(ForearmState.Base1Cube1, done())),
                                                done()))))));
    }
}