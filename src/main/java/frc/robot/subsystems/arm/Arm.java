package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.statemachine.StateMachine;
import frc.robot.util.statemachine.TransitionGraph;
import frc.robot.util.statemachine.TransitionGraphNode;

public class Arm extends SubsystemBase  {
    private DoubleSolenoid topPiston;
    private DoubleSolenoid bottomPiston;

    private DoubleSolenoid hopperPiston;

    private CANSparkMax forearmMotor;

    AnalogPotentiometer forearmPotentiometer;

	StateMachine<ArmPosition> armStateMachine;
    ShoulderState currentShoulderState;
    ForearmState currentForearmState;

    ADXL345_I2C forearmIMU;
    
    public Arm() {
        topPiston = new DoubleSolenoid(Constants.Subsystems.Pneumatics.BASE_PCM, 
            PneumaticsModuleType.REVPH, Constants.Subsystems.Arm.TOP_PISTON_SOLENOID_CHANNEL, 
            Constants.Subsystems.Arm.TOP_PISTON_SOLENOID_CHANNEL + 1);

        bottomPiston = new DoubleSolenoid(Constants.Subsystems.Pneumatics.BASE_PCM, 
            PneumaticsModuleType.REVPH, Constants.Subsystems.Arm.BOTTOM_PISTON_SOLENOID_CHANNEL, 
            Constants.Subsystems.Arm.BOTTOM_PISTON_SOLENOID_CHANNEL + 1);

        hopperPiston = new DoubleSolenoid(Constants.Subsystems.Pneumatics.BASE_PCM, 
            PneumaticsModuleType.REVPH, Constants.Subsystems.Arm.HOPPER_SOLENOID_CHANNEL, 
            Constants.Subsystems.Arm.HOPPER_SOLENOID_CHANNEL + 1);

        forearmMotor = new CANSparkMax(Constants.Subsystems.Arm.FOREARM_MOTOR_CHANNEL, MotorType.kBrushless);

		forearmMotor.getPIDController().setP(Constants.Subsystems.Arm.PIDF.P);
		forearmMotor.getPIDController().setI(Constants.Subsystems.Arm.PIDF.I);
		forearmMotor.getPIDController().setD(Constants.Subsystems.Arm.PIDF.D);
		forearmMotor.getPIDController().setFF(Constants.Subsystems.Arm.PIDF.F);

        forearmMotor.setClosedLoopRampRate(0.5);

        forearmMotor.setSmartCurrentLimit(10);
        forearmMotor.setIdleMode(IdleMode.kBrake);

        //forearmPotentiometer = new AnalogPotentiometer(Constants.Subsystems.Arm.FOREARM_ENCODER_CHANNEL, 360);
        forearmIMU = new ADXL345_I2C(Port.kOnboard, Range.k16G);

        armStateMachine = new StateMachine<>(ArmPosition.Store);
        setShoulderState(ShoulderState.Base2);
        setForearmState(ForearmState.Base2Store); // BE CAREFUL HERE, THESE CALLS SYNC UP EVERYTHING!!!

        populateHopperIntakeGraph();
        populateGroundIntakeGraph();
        populateDoubleSubstationGraph();
        populateN2Graph();
        populateN1B2Graph();
        populateB1Base4Graph();
        populateBase1B1Graph();
        populateBase2N1Graph();
        populateHybridGraph();
        populateStoreGraph();

        correctForearmNeo();
    }

    @Override
    public void periodic() {
        armStateMachine.update();
    }

    double getShoulderPosition() { // Striaght up is zero, positive is towards the front
		switch (currentShoulderState) { // TODO: VERY SKETCHY!!! PLEASE REPLACE WITH SENSOR FEEDBACK
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

    double getAbsoluteEncoderForearmPosition() { // should only be used for correcting
        //return ((forearmPotentiometer.get() + 210) % 360) - 180;// - getShoulderPosition();
        return -Math.atan(forearmIMU.getZ() / forearmIMU.getY()) * (180 / Math.PI);
    }

    double getForearmPosition() {
        return -forearmMotor.getEncoder().getPosition() * Constants.Subsystems.Arm.FOREARM_MOTOR_REDUCTION * 360.0;
    }

    void correctForearmNeo() {
        forearmMotor.getEncoder().setPosition((-getAbsoluteEncoderForearmPosition() / 360.0) / Constants.Subsystems.Arm.FOREARM_MOTOR_REDUCTION);
    }

    void setTopPiston(Value value) {
        topPiston.set((value == Value.kForward) ? Value.kForward : Value.kReverse);
    }

    void setBottomPiston(Value value) {
        bottomPiston.set((value == Value.kReverse) ? Value.kForward : Value.kReverse);
    }

    void setHopperPiston(Value value) {
        hopperPiston.set((value == Value.kForward) ? Value.kForward : Value.kReverse);
    }

    long lastShoulderActuationTime = System.currentTimeMillis();

    public void setShoulderState(ShoulderState newState){
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

    boolean isAtShoulderState(ShoulderState state) {
        return (System.currentTimeMillis() - lastShoulderActuationTime) >= 3000; // TODO: check sensors
    }

    public void setForearmState(ForearmState newState) {
        double rotationsNeeded = -Constants.Subsystems.Arm.FOREARM_POSITION.get(newState) / 360.0 / Constants.Subsystems.Arm.FOREARM_MOTOR_REDUCTION;

        forearmMotor.getPIDController().setOutputRange(-0.5, 0.5);
        forearmMotor.getPIDController().setReference(rotationsNeeded, ControlType.kPosition);
    
        currentForearmState = newState;

        System.out.println("Forearm: " + newState.toString() + " Angle: " + Constants.Subsystems.Arm.FOREARM_POSITION.get(newState));
    }

    public void directDriveArm(double power) {
        forearmMotor.set(power);
    }

    boolean isAtForearmState(ForearmState state) {
        boolean there = Math.abs(getForearmPosition() - Constants.Subsystems.Arm.FOREARM_POSITION.get(currentForearmState)) <= Constants.Subsystems.Arm.FOREARM_TARGET_POSITION_TOLERANCE;
        
        if (there) System.out.println("Forearm Hit: " + getForearmPosition());

        return there;
    }

    long lastHopperActuationTime = System.currentTimeMillis();
    
    void setHopperState(HopperState newState) {
        hopperPiston.set((newState == HopperState.Extended) ? Value.kForward : Value.kReverse);

        lastHopperActuationTime = System.currentTimeMillis();

        System.out.println("Hopper: " + newState.toString());
    }

    boolean isAtHopperState(HopperState state) {
        return (System.currentTimeMillis() - lastHopperActuationTime) >= 500; // we don't actually have a sensor for this, might as well have a way just in case though
    }

    public void setArmState(ArmPosition newState) {
        armStateMachine.transitionTo(newState);
    }

    //region Helper Functions for building the State Machine

    TransitionGraphNode shoulderMovement(ShoulderState newState, TransitionGraphNode next) {
        return new TransitionGraphNode(
            () -> setShoulderState(newState), 
            () -> isAtShoulderState(newState), next);
    }

    TransitionGraphNode forearmMovement(ForearmState newState, TransitionGraphNode next) {
        return new TransitionGraphNode(
            () -> setForearmState(newState), 
            () -> isAtForearmState(newState), next);
    }

    TransitionGraphNode hopperMovement(HopperState newState, TransitionGraphNode next) {
        return new TransitionGraphNode(
            () -> setHopperState(newState), 
            () -> isAtHopperState(newState), next);
    }

    TransitionGraphNode parallelMovement(ForearmState newForearmState, ShoulderState newShoulderState, TransitionGraphNode next) {
        return new TransitionGraphNode(
            () -> { setForearmState(newForearmState); setShoulderState(newShoulderState); }, 
            () -> isAtForearmState(newForearmState) && isAtShoulderState(newShoulderState), next);
    }

    TransitionGraphNode parallelMovement(ForearmState newForearmState, HopperState newHopperState, TransitionGraphNode next) {
        return new TransitionGraphNode(
            () -> { setForearmState(newForearmState); setHopperState(newHopperState); }, 
            () -> isAtForearmState(newForearmState) && isAtHopperState(newHopperState), next);
    }

    TransitionGraphNode parallelMovement(ShoulderState newShoulderState, HopperState newHopperState, TransitionGraphNode next) {
        return new TransitionGraphNode(
            () -> { setShoulderState(newShoulderState); setHopperState(newHopperState); }, 
            () -> isAtShoulderState(newShoulderState) && isAtHopperState(newHopperState), next);
    }

    TransitionGraphNode parallelMovement(ForearmState newForearmState, ShoulderState newShoulderState, HopperState newHopperState, TransitionGraphNode next) {
        return new TransitionGraphNode(
            () -> { setForearmState(newForearmState); setShoulderState(newShoulderState); setHopperState(newHopperState); }, 
            () -> isAtForearmState(newForearmState) && isAtShoulderState(newShoulderState) && isAtHopperState(newHopperState), next);
    }

    TransitionGraphNode done() {
        return new TransitionGraphNode(
            () -> System.out.println("--------------------"), 
            () -> true, null);
    }

    //endregion

    void populateHopperIntakeGraph() {
        setForearmState(ForearmState.Base1HopperGrab);
        armStateMachine.addBehaviour(ArmPosition.HopperIntake, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.HopperIntake, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4,
                forearmMovement(ForearmState.Base4DoubleSub, 
                hopperMovement(HopperState.Extended, 
                shoulderMovement(ShoulderState.Base2, 
                forearmMovement(ForearmState.Base2Store, 
                hopperMovement(HopperState.Retracted, 
                shoulderMovement(ShoulderState.Base1, 
                forearmMovement(ForearmState.Base1HopperGrab, 
                done()))))))), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.Base1HopperGrab, 
                shoulderMovement(ShoulderState.Base1, 
                done())),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.Base1HopperGrab, 
                    done(), 
                    
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Base1HopperGrab, 
                    shoulderMovement(ShoulderState.Base1, 
                    done()))))),
            done())))
        ));
    }

    void populateGroundIntakeGraph() {
        setForearmState(ForearmState.Base1Ground);
        armStateMachine.addBehaviour(ArmPosition.GroundIntake, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.GroundIntake, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4,
                forearmMovement(ForearmState.Base4DoubleSub, 
                hopperMovement(HopperState.Extended, 
                shoulderMovement(ShoulderState.Base2, 
                forearmMovement(ForearmState.Base2Store, 
                hopperMovement(HopperState.Retracted, 
                forearmMovement(ForearmState.Intermediate, 
                shoulderMovement(ShoulderState.Base1, 
                forearmMovement(ForearmState.Base1Ground, 
                done())))))))), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                hopperMovement(HopperState.Retracted,
                forearmMovement(ForearmState.Intermediate, 
                shoulderMovement(ShoulderState.Base1, 
                forearmMovement(ForearmState.Base1Ground, 
                done())))),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.Base1HopperGrab, 
                    hopperMovement(HopperState.Retracted,
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base1, 
                    forearmMovement(ForearmState.Base1Ground, 
                    done()))))), 
                    
                    forearmMovement(ForearmState.Base1Ground, 
                    done())),
            done())))
        ));
    }

    void populateDoubleSubstationGraph() {
        setForearmState(ForearmState.Base4DoubleSub);
        armStateMachine.addBehaviour(ArmPosition.DoubleSubstation, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.DoubleSubstation, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                forearmMovement(ForearmState.Base4DoubleSub, 
                done()), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.Base2Store, 
                hopperMovement(HopperState.Extended, 
                forearmMovement(ForearmState.Base4DoubleSub, 
                shoulderMovement(ShoulderState.Base4, 
                done())))),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.Base1HopperGrab, 
                    parallelMovement(ForearmState.Base2Store, ShoulderState.Base2, 
                    hopperMovement(HopperState.Extended, 
                    forearmMovement(ForearmState.Base4DoubleSub, 
                    shoulderMovement(ShoulderState.Base4, 
                    done())))), 
                    
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Base2Store,
                    hopperMovement(HopperState.Extended, 
                    forearmMovement(ForearmState.Base4DoubleSub, 
                    shoulderMovement(ShoulderState.Base4, 
                    done()))))))),
            done())))
        ));
    }

    void populateN2Graph() {
        setForearmState(ForearmState.Base4Cone2);
        armStateMachine.addBehaviour(ArmPosition.N2, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.N2, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                forearmMovement(ForearmState.Base4Cone2, 
                done()), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.Base2Store, 
                hopperMovement(HopperState.Extended, 
                forearmMovement(ForearmState.Base4DoubleSub, 
                shoulderMovement(ShoulderState.Base4, 
                forearmMovement(ForearmState.Base4Cone2, 
                done()))))),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.Base1HopperGrab, 
                    parallelMovement(ForearmState.Base2Store, ShoulderState.Base2, 
                    hopperMovement(HopperState.Extended, 
                    forearmMovement(ForearmState.Base4DoubleSub, 
                    shoulderMovement(ShoulderState.Base4, 
                    forearmMovement(ForearmState.Base4Cone2, 
                    done()))))), 
                    
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Base2Store,
                    hopperMovement(HopperState.Extended, 
                    forearmMovement(ForearmState.Base4DoubleSub, 
                    shoulderMovement(ShoulderState.Base4, 
                    forearmMovement(ForearmState.Base4Cone2, 
                    done())))))))),
            done())))
        ));
    }

    void populateN1B2Graph() {
        setForearmState(ForearmState.Base4Cube2);
        armStateMachine.addBehaviour(ArmPosition.N1B2, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.N1B2, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                forearmMovement(ForearmState.Base4Cube2, 
                done()), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.Base2Store, 
                hopperMovement(HopperState.Extended, 
                forearmMovement(ForearmState.Base4DoubleSub, 
                shoulderMovement(ShoulderState.Base4, 
                forearmMovement(ForearmState.Base4Cube2, 
                done()))))),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.Base1HopperGrab, 
                    parallelMovement(ForearmState.Base2Store, ShoulderState.Base2, 
                    hopperMovement(HopperState.Extended, 
                    forearmMovement(ForearmState.Base4DoubleSub, 
                    shoulderMovement(ShoulderState.Base4, 
                    forearmMovement(ForearmState.Base4Cube2, 
                    done()))))), 
                    
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Base2Store,
                    hopperMovement(HopperState.Extended, 
                    forearmMovement(ForearmState.Base4DoubleSub, 
                    shoulderMovement(ShoulderState.Base4, 
                    forearmMovement(ForearmState.Base4Cube2, 
                    done())))))))),
            done())))
        ));
    }

    void populateB1Base4Graph() {
        setForearmState(ForearmState.Base4Cube1);
        armStateMachine.addBehaviour(ArmPosition.B1Base4, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.B1Base4, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                forearmMovement(ForearmState.Base4Cube1, 
                done()), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.Base2Store, 
                hopperMovement(HopperState.Extended, 
                forearmMovement(ForearmState.Base4DoubleSub, 
                shoulderMovement(ShoulderState.Base4, 
                forearmMovement(ForearmState.Base4Cube1, 
                done()))))),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.Base1HopperGrab, 
                    parallelMovement(ForearmState.Base2Store, ShoulderState.Base2, 
                    hopperMovement(HopperState.Extended, 
                    forearmMovement(ForearmState.Base4DoubleSub, 
                    shoulderMovement(ShoulderState.Base4, 
                    forearmMovement(ForearmState.Base4Cube1, 
                    done()))))), 
                    
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Base2Store,
                    hopperMovement(HopperState.Extended, 
                    forearmMovement(ForearmState.Base4DoubleSub, 
                    shoulderMovement(ShoulderState.Base4, 
                    forearmMovement(ForearmState.Base4Cube1, 
                    done())))))))),
            done())))
        ));
    }

    void populateBase1B1Graph() {
        setForearmState(ForearmState.Base1Cube1);
        armStateMachine.addBehaviour(ArmPosition.Base1B1, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.Base1B1, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                hopperMovement(HopperState.Extended, 
                forearmMovement(ForearmState.Base4DoubleSub,
                shoulderMovement(ShoulderState.Base2,
                forearmMovement(ForearmState.Base2Store,
                hopperMovement(HopperState.Retracted, 
                forearmMovement(ForearmState.Base1Cube1, 
                shoulderMovement(ShoulderState.Base1, 
                done()))))))), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.Base1Cube1, 
                shoulderMovement(ShoulderState.Base1, 
                done())),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.Base1HopperGrab, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Base1Cube1, 
                    shoulderMovement(ShoulderState.Base1, 
                    done()))), 
                    
                    forearmMovement(ForearmState.Base1Cube1, 
                    done())),
            done())))
        ));
    }

    void populateBase2N1Graph() {
        setForearmState(ForearmState.Base2Cone1);
        armStateMachine.addBehaviour(ArmPosition.Base2N1, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.Base2N1, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                hopperMovement(HopperState.Extended, 
                forearmMovement(ForearmState.Base4DoubleSub, 
                shoulderMovement(ShoulderState.Base2, 
                forearmMovement(ForearmState.Base2Store,
                hopperMovement(HopperState.Retracted, 
                forearmMovement(ForearmState.Base2Cone1, 
                done())))))), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.Base2Cone1,
                done()),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.Base1HopperGrab, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Base2Cone1, 
                    done())), 
                    
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Base2Cone1,
                    done())))),
            done())))
        ));
    }

    void populateHybridGraph() {
        setForearmState(ForearmState.Base1Hybrid);
        armStateMachine.addBehaviour(ArmPosition.Hybrid, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.Hybrid, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                hopperMovement(HopperState.Extended,
                forearmMovement(ForearmState.Base4DoubleSub, 
                shoulderMovement(ShoulderState.Base2, 
                forearmMovement(ForearmState.Base2Store, 
                hopperMovement(HopperState.Retracted, 
                forearmMovement(ForearmState.Intermediate, 
                shoulderMovement(ShoulderState.Base1, 
                forearmMovement(ForearmState.Base1Hybrid, 
                done())))))))), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.Intermediate, 
                shoulderMovement(ShoulderState.Base1, 
                forearmMovement(ForearmState.Base1Hybrid, 
                done()))),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.Base1HopperGrab, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base1, 
                    forearmMovement(ForearmState.Base1Hybrid, 
                    done())))), 
                    
                    forearmMovement(ForearmState.Base1Hybrid, 
                    done())),
            done())))
        ));
    }

    void populateStoreGraph() {
        setForearmState(ForearmState.Base2Store);
        armStateMachine.addBehaviour(ArmPosition.Store, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.Store, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4,
                hopperMovement(HopperState.Extended,
                forearmMovement(ForearmState.Base4DoubleSub, 
                shoulderMovement(ShoulderState.Base2,
                forearmMovement(ForearmState.Base2Store, 
                hopperMovement(HopperState.Retracted,
                done()))))), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.Base2Store, 
                done()),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.Base1HopperGrab, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Base2Store, 
                    done())), 
                    
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Base2Store, 
                    done())))),

            done())))
        ));
    }
}
