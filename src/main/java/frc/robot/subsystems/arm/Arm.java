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
        /*
        setMovementAdderArmPosition(ArmPosition.HopperIntake);

        addOneWayBranch( "start", "branch4 step1",
            () -> currentShoulderState == ShoulderState.Base4);
        addOneWayBranch( "start", "branch2 step1",
            () -> currentShoulderState == ShoulderState.Base2);
        addOneWayBranch( "start", "branch1 step1",
            () -> currentShoulderState == ShoulderState.Base1);

        //branch4
        addForearmMovement("branch4 step1", "branch4 step2", ForearmState.N1B2);
        addShoulderMovement("branch4 step2", "branch4 step3", ShoulderState.Base1);
        addForearmMovement("branch4 step3", "stop", ForearmState.HopperGrab);
        //

        //branch2
        addForearmMovement("branch2 step1", "branch2 step2", ForearmState.HopperGrab);
        addShoulderMovement("branch2 step2", "stop", ShoulderState.Base1);
        //

        //branch1
        addBranch("branch1 step1", "stop", "branch1 step2", 
            () -> currentForearmState == ForearmState.HopperGrab);

        addForearmMovement("branch1 step2", "branch1 step3", ForearmState.Intermediate);
        addShoulderMovement("branch1 step3", "branch2 step1", ShoulderState.Base2);
        //
        */
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
        /*
        setMovementAdderArmPosition(ArmPosition.DoubleSubstation);

        addOneWayBranch( "start", "branch4 step1",
            () -> currentShoulderState == ShoulderState.Base4);
        addOneWayBranch( "start", "branch2 step1",
            () -> currentShoulderState == ShoulderState.Base2);
        addOneWayBranch( "start", "branch1 step1",
            () -> currentShoulderState == ShoulderState.Base1);
            
        //branch4
        addForearmMovement("branch4 step1", "stop", ForearmState.DoubleSub);
        //

        //branch2
        addForearmMovement("branch2 step1", "branch2 step2", ForearmState.DoubleSub);
        addHopperMovement("branch2 step2", "branch2 step3", true);
        addParallelMovement("branch2 step3", "branch2 step4", ForearmState.DoubleSub, ShoulderState.Base4);
        addHopperMovement("branch2 step4", "stop", false);
        //

        //branch1
        addBranch("branch1 step1", "branch1 branch1 step1", "branch1 branch2 step1", 
            () -> currentForearmState == ForearmState.HopperGrab);

        //branch1 branch1
        addParallelMovement("branch1 branch1 step1", "branch2 step2", ForearmState.Store, ShoulderState.Base2);
        //

        //branch1 branch2
        addForearmMovement("branch1 branch2 step1", "branch1 branch2 step2", ForearmState.Intermediate);
        addParallelMovement("branch1 branch2 step2", "branch2 step2", ForearmState.Store, ShoulderState.Base2);
        //
        //
        */
    }

    void populateN2Graph() {
        /*
        setMovementAdderArmPosition(ArmPosition.N2);

        addOneWayBranch( "start", "branch4 step1",
            () -> currentShoulderState == ShoulderState.Base4);
        addOneWayBranch( "start", "branch2 step1",
            () -> currentShoulderState == ShoulderState.Base2);
        addOneWayBranch( "start", "branch1 step1",
            () -> currentShoulderState == ShoulderState.Base1);
            
        //branch4
        addForearmMovement("branch4 step1", "stop", ForearmState.N2);
        //

        //branch2
        addForearmMovement("branch2 step1", "branch2 step2", ForearmState.N2);
        addHopperMovement("branch2 step2", "branch2 step3", true);
        addParallelMovement("branch2 step3", "branch2 step4", ForearmState.N2, ShoulderState.Base4);
        addHopperMovement("branch2 step4", "stop", false);
        //

        //branch1
        addBranch("branch1 step1", "branch1 branch1 step1", "branch1 branch2 step1", 
            () -> currentForearmState == ForearmState.HopperGrab);

        //branch1 branch1
        addParallelMovement("branch1 branch1 step1", "branch2 step2", ForearmState.Store, ShoulderState.Base2);
        //

        //branch1 branch2
        addForearmMovement("branch1 branch2 step1", "branch1 branch2 step2", ForearmState.Intermediate);
        addParallelMovement("branch1 branch2 step2", "branch2 step2", ForearmState.Store, ShoulderState.Base2);
        //
        //
        //
        */
    }

    void populateN1B2Graph() {
        /*
        setMovementAdderArmPosition(ArmPosition.N1B2);

        addOneWayBranch( "start", "branch4 step1",
            () -> currentShoulderState == ShoulderState.Base4);
        addOneWayBranch( "start", "branch2 step1",
            () -> currentShoulderState == ShoulderState.Base2);
        addOneWayBranch( "start", "branch1 step1",
            () -> currentShoulderState == ShoulderState.Base1);
            
        //branch4
        addForearmMovement("branch4 step1", "stop", ForearmState.N1B2);
        //

        //branch2
        addForearmMovement("branch2 step1", "branch2 step2", ForearmState.N1B2);
        addHopperMovement("branch2 step2", "branch2 step3", true);
        addParallelMovement("branch2 step3", "branch2 step4", ForearmState.N1B2, ShoulderState.Base4);
        addHopperMovement("branch2 step4", "stop", false);
        //

        //branch1
        addBranch("branch1 step1", "branch1 branch1 step1", "branch1 branch2 step1", 
            () -> currentForearmState == ForearmState.HopperGrab);

        //branch1 branch1
        addParallelMovement("branch1 branch1 step1", "branch2 step2", ForearmState.Store, ShoulderState.Base2);
        //

        //branch1 branch2
        addForearmMovement("branch1 branch2 step1", "branch1 branch2 step2", ForearmState.Intermediate);
        addParallelMovement("branch1 branch2 step2", "branch2 step2", ForearmState.Store, ShoulderState.Base2);
        //
        //
        //
        */
    }

    void populateB1Base4Graph() {
        /*
        setMovementAdderArmPosition(ArmPosition.B1Base4);

        addOneWayBranch( "start", "branch4 step1",
            () -> currentShoulderState == ShoulderState.Base4);
        addOneWayBranch( "start", "branch2 step1",
            () -> currentShoulderState == ShoulderState.Base2);
        addOneWayBranch( "start", "branch1 step1",
            () -> currentShoulderState == ShoulderState.Base1);
            
        //branch4
        addForearmMovement("branch4 step1", "stop", ForearmState.B1);
        //

        //branch2
        addForearmMovement("branch2 step1", "branch2 step2", ForearmState.B1);
        addHopperMovement("branch2 step2", "branch2 step3", true);
        addParallelMovement("branch2 step3", "branch2 step4", ForearmState.B1, ShoulderState.Base4);
        addHopperMovement("branch2 step4", "stop", false);
        //

        //branch1
        addBranch("branch1 step1", "branch1 branch1 step1", "branch1 branch2 step1", 
            () -> currentForearmState == ForearmState.HopperGrab);

        //branch1 branch1
        addParallelMovement("branch1 branch1 step1", "branch2 step2", ForearmState.Store, ShoulderState.Base2);
        //

        //branch1 branch2
        addForearmMovement("branch1 branch2 step1", "branch1 branch2 step2", ForearmState.Intermediate);
        addParallelMovement("branch1 branch2 step2", "branch2 step2", ForearmState.Store, ShoulderState.Base2);
        //
        //
        //
        */
    }

    void populateBase1B1Graph() {
        /*
        setMovementAdderArmPosition(ArmPosition.Base1B1);

        addOneWayBranch( "start", "branch4 step1",
            () -> currentShoulderState == ShoulderState.Base4);
        addOneWayBranch( "start", "branch2 step1",
            () -> currentShoulderState == ShoulderState.Base2);
        addOneWayBranch( "start", "branch1 step1",
            () -> currentShoulderState == ShoulderState.Base1);

        //branch4
        addForearmMovement("branch4 step1", "branch4 step2", ForearmState.Base1B1);
        addShoulderMovement("branch4 step2", "branch4 step3", ShoulderState.Base1);
        addForearmMovement("branch4 step3", "stop", ForearmState.Base1B1);
        //

        //branch2
        addForearmMovement("branch2 step1", "branch2 step2", ForearmState.Base1B1);
        addShoulderMovement("branch2 step2", "stop", ShoulderState.Base1);
        //

        //branch1
        addBranch("branch1 step1", "branch1 branch2 step1", "branch1 branch1 step1", 
            () -> currentForearmState == ForearmState.HopperGrab);


        //branch1 branch1
        addForearmMovement("branch1 branch1 step1", "branch1 branch1 step2", ForearmState.Intermediate);
        addShoulderMovement("branch1 branch1 step2", "stop", ShoulderState.Base2);
        //

        //branch1 branch2
        addShoulderMovement("branch1 branch2 step1", "branch1 branch2 step2", ShoulderState.Base2);
        addForearmMovement("branch1 branch2 step2", "branch1 branch2 step3", ForearmState.Intermediate);
        addShoulderMovement("branch1 branch2 step3", "branch1 branch2 step4", ShoulderState.Base1);
        addForearmMovement("branch1 branch2 step4", "stop", ForearmState.Base1B1);
        //
        */
    }

    void populateHybridGraph() {
        /*
        setMovementAdderArmPosition(ArmPosition.Hybrid);

        addOneWayBranch( "start", "branch4 step1",
            () -> currentShoulderState == ShoulderState.Base4);
        addOneWayBranch( "start", "branch4 step4",
            () -> currentShoulderState == ShoulderState.Base2);
        addOneWayBranch( "start", "branch1 step1",
            () -> currentShoulderState == ShoulderState.Base1);

        //branch4
        addParallelMovement("branch4 step1", "branch4 step2", ForearmState.Store, true);
        addShoulderMovement("branch4 step2", "branch4 step3", ShoulderState.Base2);
        addHopperMovement("branch4 step3", "branch4 step4", false);
        addForearmMovement("branch4 step4", "branch4 step5", ForearmState.Intermediate);
        addShoulderMovement("branch4 step5", "branch4 step6", ShoulderState.Base1);
        addForearmMovement("branch4 step6", "stop", ForearmState.Hybrid);
        //

        //branch1
        addBranch("branch1 step1", "branch1 branch2 step1", "branch1 branch1 step1", 
            () -> currentForearmState == ForearmState.HopperGrab);


        //branch1 branch1
        addForearmMovement("branch1 branch1 step1", "stop", ForearmState.Hybrid);
        //

        //branch1 branch2
        addShoulderMovement("branch1 branch2 step1", "branch4 step4", ShoulderState.Base2);
        //
        //
        */
    }

    void populateStoreGraph() {
        /*
        setMovementAdderArmPosition(ArmPosition.Store);

        addOneWayBranch( "start", "branch4 step1",
            () -> currentShoulderState == ShoulderState.Base4);
        addOneWayBranch( "start", "branch2 step1",
            () -> currentShoulderState == ShoulderState.Base2);
        addOneWayBranch( "start", "branch1 step1",
            () -> currentShoulderState == ShoulderState.Base1);

        //branch4
        addParallelMovement("branch4 step1", "branch4 step2", ForearmState.Store, true);
        addShoulderMovement("branch4 step2", "branch4 step3", ShoulderState.Base2);
        addHopperMovement("branch4 step3", "stop", false);
        //

        //branch2
        addForearmMovement("branch2 step1", "stop", ForearmState.Store);
        //

        //branch1
        addBranch("branch1 step1", "branch1 branch2 step1", "branch1 branch1 step1", 
            () -> currentForearmState == ForearmState.HopperGrab);


        //branch1 branch1
        addForearmMovement("branch1 branch1 step1", "branch1 branch2 step1", ForearmState.Intermediate);
        //

        //branch1 branch2
        addShoulderMovement("branch1 branch2 step1", "branch1 branch2 step2", ShoulderState.Base2);
        addForearmMovement("branch1 branch2 step2", "stop", ForearmState.Store);
        //
        //
        */
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
