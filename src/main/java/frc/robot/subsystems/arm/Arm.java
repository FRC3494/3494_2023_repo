package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

	StateMachine<ArmPosition> armStateMachine;
    ShoulderState currentShoulderState;
    ForearmState currentForearmState;

    ArmPosition movementAdderArmPosition;
    
    public Arm(){
        topPiston = new DoubleSolenoid(Constants.Subsystems.Pneumatics.BASE_PCM, 
            PneumaticsModuleType.REVPH, Constants.Subsystems.Arm.TOP_PISTON_SOLENOID_CHANNEL, 
            Constants.Subsystems.Arm.TOP_PISTON_SOLENOID_CHANNEL + 1);

        bottomPiston = new DoubleSolenoid(Constants.Subsystems.Pneumatics.BASE_PCM, 
            PneumaticsModuleType.REVPH, Constants.Subsystems.Arm.BOTTOM_PISTON_SOLENOID_CHANNEL, 
            Constants.Subsystems.Arm.BOTTOM_PISTON_SOLENOID_CHANNEL + 1);

        hopperPiston = new DoubleSolenoid(Constants.Subsystems.Pneumatics.BASE_PCM, 
            PneumaticsModuleType.REVPH, Constants.Subsystems.Arm.HOPPER_SOLENOID_CHANNEL, 
            Constants.Subsystems.Arm.HOPPER_SOLENOID_CHANNEL + 1);

        forearmMotor = new CANSparkMax(Constants.Subsystems.Arm.ARM_MOTOR_CHANNEL, MotorType.kBrushless);

		forearmMotor.getPIDController().setP(Constants.Subsystems.Arm.PIDF.P);
		forearmMotor.getPIDController().setP(Constants.Subsystems.Arm.PIDF.I);
		forearmMotor.getPIDController().setP(Constants.Subsystems.Arm.PIDF.D);
		forearmMotor.getPIDController().setP(Constants.Subsystems.Arm.PIDF.F);

        armStateMachine = new StateMachine<>(ArmPosition.Store);
        setShoulderState(ShoulderState.Base2);
        setForearmState(ForearmState.Store); // BE CAREFUL HERE, THESE CALLS SYNC UP EVERYTHING!!!

        populateHopperIntakeGraph();
        populateGroundIntakeGraph();
        populateDoubleSubstationGraph();
        populateN2Graph();
        populateN1B2Graph();
        populateB1Base4Graph();
        populateBase1B1Graph();
        populateHybridGraph();
        populateStoreGraph();
    }

    @Override
    public void periodic() {
        armStateMachine.update();
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

    void setShoulderState(ShoulderState newState){
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
        return (System.currentTimeMillis() - lastShoulderActuationTime) >= 1000; // #TODO: check sensors
    }

    long lastForearmActuationTime = System.currentTimeMillis();

    void setForearmState(ForearmState newState) {
        forearmMotor.getPIDController().setReference(Constants.Subsystems.Arm.FOREARM_POSITION.get(newState), ControlType.kPosition);
    
        currentForearmState = newState;

        lastForearmActuationTime = System.currentTimeMillis();

        System.out.println("Forearm: " + newState.toString());
    }

    boolean isAtForearmState(ForearmState state) {
        return (System.currentTimeMillis() - lastForearmActuationTime) >= 1000; // #TODO: check sensors
    }

    long lastHopperActuationTime = System.currentTimeMillis();
    
    void setHopperState(HopperState newState) {
        hopperPiston.set((newState == HopperState.Extended) ? Value.kForward : Value.kReverse);

        lastHopperActuationTime = System.currentTimeMillis();

        System.out.println("Hopper: " + newState.toString());
    }

    boolean isAtHopperState(HopperState state) {
        return (System.currentTimeMillis() - lastHopperActuationTime) >= 1000; // we don't actually have a sensor for this, might as well have a way just in case though
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

    //endregion

    void populateHopperIntakeGraph() {
        armStateMachine.addBehaviour(ArmPosition.HopperIntake, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.HopperIntake, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                forearmMovement(ForearmState.N1B2, 
                shoulderMovement(ShoulderState.Base1, 
                forearmMovement(ForearmState.HopperGrab, null))), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.HopperGrab, 
                shoulderMovement(ShoulderState.Base1, null)),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.HopperGrab, 
                    null, 
                    
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.HopperGrab, 
                    shoulderMovement(ShoulderState.Base1, null))))),
            null)))
        ));
    }

    void populateGroundIntakeGraph() {
        armStateMachine.addBehaviour(ArmPosition.GroundIntake, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.GroundIntake, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                parallelMovement(ForearmState.Store, HopperState.Extended, 
                shoulderMovement(ShoulderState.Base2,
                hopperMovement(HopperState.Retracted,
                forearmMovement(ForearmState.Intermediate, 
                shoulderMovement(ShoulderState.Base1, 
                forearmMovement(ForearmState.Ground, null)))))), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                hopperMovement(HopperState.Retracted,
                forearmMovement(ForearmState.Intermediate, 
                shoulderMovement(ShoulderState.Base1, 
                forearmMovement(ForearmState.Ground, null)))),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.HopperGrab, 
                    shoulderMovement(ShoulderState.Base2, 
                    hopperMovement(HopperState.Retracted,
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base1, 
                    forearmMovement(ForearmState.Ground, null))))), 
                    
                    forearmMovement(ForearmState.Ground, null)),
            null)))
        ));
    }

    void populateDoubleSubstationGraph() {
        armStateMachine.addBehaviour(ArmPosition.DoubleSubstation, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.DoubleSubstation, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                forearmMovement(ForearmState.DoubleSub, null), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.DoubleSub, 
                hopperMovement(HopperState.Extended, 
                parallelMovement(ForearmState.DoubleSub, ShoulderState.Base4, 
                hopperMovement(HopperState.Retracted, null)))),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.HopperGrab, 
                    parallelMovement(ForearmState.Store, ShoulderState.Base2, 
                    hopperMovement(HopperState.Extended, 
                    parallelMovement(ForearmState.DoubleSub, ShoulderState.Base4, 
                    hopperMovement(HopperState.Retracted, null)))), 
                    
                    forearmMovement(ForearmState.Intermediate, 
                    parallelMovement(ForearmState.Store, ShoulderState.Base2, 
                    hopperMovement(HopperState.Extended, 
                    parallelMovement(ForearmState.DoubleSub, ShoulderState.Base4, 
                    hopperMovement(HopperState.Retracted, null)))))),
            null)))
        ));
    }

    void populateN2Graph() {
        armStateMachine.addBehaviour(ArmPosition.N2, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.N2, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                forearmMovement(ForearmState.N2, null), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.N2, 
                hopperMovement(HopperState.Extended, 
                parallelMovement(ForearmState.N2, ShoulderState.Base4, 
                hopperMovement(HopperState.Retracted, null)))),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.HopperGrab, 
                    parallelMovement(ForearmState.Store, ShoulderState.Base2, 
                    hopperMovement(HopperState.Extended, 
                    parallelMovement(ForearmState.N2, ShoulderState.Base4, 
                    hopperMovement(HopperState.Retracted, null)))), 
                    
                    forearmMovement(ForearmState.Intermediate, 
                    parallelMovement(ForearmState.Store, ShoulderState.Base2, 
                    hopperMovement(HopperState.Extended, 
                    parallelMovement(ForearmState.N2, ShoulderState.Base4, 
                    hopperMovement(HopperState.Retracted, null)))))),
            null)))
        ));
    }

    void populateN1B2Graph() {
        armStateMachine.addBehaviour(ArmPosition.N1B2, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.N1B2, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                forearmMovement(ForearmState.N1B2, null), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.N1B2, 
                hopperMovement(HopperState.Extended, 
                parallelMovement(ForearmState.N1B2, ShoulderState.Base4, 
                hopperMovement(HopperState.Retracted, null)))),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.HopperGrab, 
                    parallelMovement(ForearmState.Store, ShoulderState.Base2, 
                    hopperMovement(HopperState.Extended, 
                    parallelMovement(ForearmState.N1B2, ShoulderState.Base4, 
                    hopperMovement(HopperState.Retracted, null)))), 
                    
                    forearmMovement(ForearmState.Intermediate, 
                    parallelMovement(ForearmState.Store, ShoulderState.Base2, 
                    hopperMovement(HopperState.Extended, 
                    parallelMovement(ForearmState.N1B2, ShoulderState.Base4, 
                    hopperMovement(HopperState.Retracted, null)))))),
            null)))
        ));
    }

    void populateB1Base4Graph() {
        armStateMachine.addBehaviour(ArmPosition.B1Base4, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.B1Base4, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                forearmMovement(ForearmState.B1, null), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.B1, 
                hopperMovement(HopperState.Extended, 
                parallelMovement(ForearmState.B1, ShoulderState.Base4, 
                hopperMovement(HopperState.Retracted, null)))),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.HopperGrab, 
                    parallelMovement(ForearmState.Store, ShoulderState.Base2, 
                    hopperMovement(HopperState.Extended, 
                    parallelMovement(ForearmState.B1, ShoulderState.Base4, 
                    hopperMovement(HopperState.Retracted, null)))), 
                    
                    forearmMovement(ForearmState.Intermediate, 
                    parallelMovement(ForearmState.Store, ShoulderState.Base2, 
                    hopperMovement(HopperState.Extended, 
                    parallelMovement(ForearmState.B1, ShoulderState.Base4, 
                    hopperMovement(HopperState.Retracted, null)))))),
            null)))
        ));
    }

    void populateBase1B1Graph() {
        armStateMachine.addBehaviour(ArmPosition.Base1B1, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.Base1B1, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                forearmMovement(ForearmState.Base1B1, 
                shoulderMovement(ShoulderState.Base1, 
                forearmMovement(ForearmState.Base1B1, null))), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.Base1B1, 
                shoulderMovement(ShoulderState.Base1, null)),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.HopperGrab, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base1, 
                    forearmMovement(ForearmState.Base1B1, null)))), 
                    
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base2, null))),
            null)))
        ));
    }

    void populateHybridGraph() {
        armStateMachine.addBehaviour(ArmPosition.Hybrid, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.Hybrid, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                parallelMovement(ForearmState.Store, HopperState.Extended, 
                shoulderMovement(ShoulderState.Base2, 
                hopperMovement(HopperState.Retracted, 
                forearmMovement(ForearmState.Intermediate, 
                shoulderMovement(ShoulderState.Base1, 
                forearmMovement(ForearmState.Hybrid, null)))))), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.Intermediate, 
                shoulderMovement(ShoulderState.Base1, 
                forearmMovement(ForearmState.Hybrid, null))),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.HopperGrab, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base1, 
                    forearmMovement(ForearmState.Hybrid, null)))), 
                    
                    forearmMovement(ForearmState.Hybrid, null)),
            null)))
        ));
    }

    void populateStoreGraph() {
        armStateMachine.addBehaviour(ArmPosition.Store, () -> {});
        armStateMachine.addTransitionGraph(null, ArmPosition.Store, new TransitionGraph(
            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base4, 
                parallelMovement(ForearmState.Store, HopperState.Extended, 
                shoulderMovement(ShoulderState.Base2,
                hopperMovement(HopperState.Retracted,null))), 

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base2, 
                forearmMovement(ForearmState.Store, null),

            new TransitionGraphNode(() -> currentShoulderState == ShoulderState.Base1, 
                new TransitionGraphNode(() -> currentForearmState == ForearmState.HopperGrab, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Store, null)), 
                    
                    forearmMovement(ForearmState.Intermediate, 
                    shoulderMovement(ShoulderState.Base2, 
                    forearmMovement(ForearmState.Store, null)))),

            null)))
        ));
    }
}
