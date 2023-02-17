package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Conditions;
import frc.robot.util.StateMachine;

public class Arm extends SubsystemBase  {
    private DoubleSolenoid basePiston1;
    private DoubleSolenoid basePiston2;

    private DoubleSolenoid hopperPiston;

    private CANSparkMax forearmMotor;

	StateMachine<ArmState> armStateMachine;
    ShoulderState currentShoulderState;
    ForearmState currenForearmState;

    ArmPosition movementAdderArmPosition;
    
    public Arm(){
        basePiston1 = new DoubleSolenoid(Constants.Subsystems.Arm.BASE_PCM, 
            PneumaticsModuleType.REVPH, Constants.Subsystems.Arm.PISTON1_SOLENOID_CHANNEL, 
            Constants.Subsystems.Arm.PISTON1_SOLENOID_CHANNEL + 1);

        basePiston2 = new DoubleSolenoid(Constants.Subsystems.Arm.BASE_PCM, 
            PneumaticsModuleType.REVPH, Constants.Subsystems.Arm.PISTON2_SOLENOID_CHANNEL, 
            Constants.Subsystems.Arm.PISTON2_SOLENOID_CHANNEL + 1);

        hopperPiston = new DoubleSolenoid(Constants.Subsystems.Arm.BASE_PCM, 
            PneumaticsModuleType.REVPH, Constants.Subsystems.Arm.HOPPER_SOLENOID_CHANNEL, 
            Constants.Subsystems.Arm.HOPPER_SOLENOID_CHANNEL + 1);

        forearmMotor = new CANSparkMax(Constants.Subsystems.Arm.ARM_MOTOR_CHANNEL, MotorType.kBrushless);

		forearmMotor.getPIDController().setP(Constants.Subsystems.Arm.PIDF.P);
		forearmMotor.getPIDController().setP(Constants.Subsystems.Arm.PIDF.I);
		forearmMotor.getPIDController().setP(Constants.Subsystems.Arm.PIDF.D);
		forearmMotor.getPIDController().setP(Constants.Subsystems.Arm.PIDF.F);

        armStateMachine = new StateMachine<>(new ArmState(ArmPosition.Store, "stop"));
        setShoulderState(ShoulderState.Base2);
        setForearmState(ForearmState.Cone1Left); // BE CAREFUL HERE, THESE CALLS SYNC UP EVERYTHING!!!

        populateHopperIntakeGraph();
    }

    @Override
    public void periodic() {
        armStateMachine.update();
    }

    void setShoulderState(ShoulderState newState){
		switch (newState) {
			case Base1:
            	basePiston1.set(Value.kReverse);
            	basePiston2.set(Value.kReverse);
				break;
			case Base2:
            	basePiston1.set(Value.kForward);
            	basePiston2.set(Value.kReverse);
				break;
			case Base3:
            	basePiston1.set(Value.kReverse);
            	basePiston2.set(Value.kForward);
				break;
			case Base4:
            	basePiston1.set(Value.kForward);
            	basePiston2.set(Value.kForward);
				break;
		}

        currentShoulderState = newState;
    }

    boolean isAtShoulderState(ShoulderState state) {
        return true; // #TODO: check sensors
    }

    void setForearmState(ForearmState newState) {
        forearmMotor.getPIDController().setReference(Constants.Subsystems.Arm.FOREARM_POSITION.get(newState), ControlType.kPosition);
    
        currenForearmState = newState;
    }

    boolean isAtForearmState(ForearmState state) {
        return true; // #TODO: check sensors
    }
    
    void setHopperState(boolean extended) {
        hopperPiston.set(extended ? Value.kForward : Value.kReverse);
    }

    boolean isAtHopperState(boolean state) {
        return true; // we don't actually have a sensor for this, might as well have a way just in case though
    }

    //region Helper Functions for building the State Machine

    public void setArmState(ArmPosition newState) {
        armStateMachine.transitionTo(new ArmState(newState, "start"));
    }

    void setMovementAdderArmPosition(ArmPosition position) {
        movementAdderArmPosition = position;
    }

    void addShoulderMovement(String on, String to, ShoulderState newState) {
        armStateMachine.addBehaviour(
            new ArmState(movementAdderArmPosition, on), 
            () -> setShoulderState(newState));

        armStateMachine.setTransitionCondition(
            new ArmState(movementAdderArmPosition, on), 
            new ArmState(movementAdderArmPosition, to), 
            () -> isAtShoulderState(newState));
    }

    void addForearmMovement(String on, String to, ForearmState newState) {
        armStateMachine.addBehaviour(
            new ArmState(movementAdderArmPosition, on), 
            () -> setForearmState(newState));

        armStateMachine.setTransitionCondition(
            new ArmState(movementAdderArmPosition, on), 
            new ArmState(movementAdderArmPosition, to), 
            () -> isAtForearmState(newState));
    }

    void addHopperMovement(String on, String to, boolean newState) {
        armStateMachine.addBehaviour(
            new ArmState(movementAdderArmPosition, on), 
            () -> setHopperState(newState));

        armStateMachine.setTransitionCondition(
            new ArmState(movementAdderArmPosition, on), 
            new ArmState(movementAdderArmPosition, to), 
            () -> isAtHopperState(newState));
    }

    void addParallelMovement(String on, String to, ForearmState newForearmState, ShoulderState newShoulderState) {
        armStateMachine.addBehaviour(
            new ArmState(movementAdderArmPosition, on), 
            () -> setForearmState(newForearmState));

        armStateMachine.addBehaviour(
            new ArmState(movementAdderArmPosition, on), 
            () -> setShoulderState(newShoulderState));

        armStateMachine.setTransitionCondition(
            new ArmState(movementAdderArmPosition, on), 
            new ArmState(movementAdderArmPosition, to), 
            () -> isAtForearmState(newForearmState) && isAtShoulderState(newShoulderState));
    }

    void addParallelMovement(String on, String to, ForearmState newForearmState, boolean newHopperState) {
        armStateMachine.addBehaviour(
            new ArmState(movementAdderArmPosition, on), 
            () -> setForearmState(newForearmState));

        armStateMachine.addBehaviour(
            new ArmState(movementAdderArmPosition, on), 
            () -> setHopperState(newHopperState));

        armStateMachine.setTransitionCondition(
            new ArmState(movementAdderArmPosition, on), 
            new ArmState(movementAdderArmPosition, to), 
            () -> isAtForearmState(newForearmState) && isAtHopperState(newHopperState));
    }

    void addParallelMovement(String on, String to, ShoulderState newForearmState, boolean newHopperState) {
        armStateMachine.addBehaviour(
            new ArmState(movementAdderArmPosition, on), 
            () -> setShoulderState(newForearmState));

        armStateMachine.addBehaviour(
            new ArmState(movementAdderArmPosition, on), 
            () -> setHopperState(newHopperState));

        armStateMachine.setTransitionCondition(
            new ArmState(movementAdderArmPosition, on), 
            new ArmState(movementAdderArmPosition, to), 
            () -> isAtShoulderState(newForearmState) && isAtHopperState(newHopperState));
    }

    void addParallelMovement(String on, String to, ForearmState newForearmState, ShoulderState newShoulderState, boolean newHopperState) {
        armStateMachine.addBehaviour(
            new ArmState(movementAdderArmPosition, on), 
            () -> setForearmState(newForearmState));

        armStateMachine.addBehaviour(
            new ArmState(movementAdderArmPosition, on), 
            () -> setShoulderState(newShoulderState));

        armStateMachine.addBehaviour(
            new ArmState(movementAdderArmPosition, on), 
            () -> setHopperState(newHopperState));

        armStateMachine.setTransitionCondition(
            new ArmState(movementAdderArmPosition, on), 
            new ArmState(movementAdderArmPosition, to), 
            () -> isAtForearmState(newForearmState) && isAtShoulderState(newShoulderState) && isAtHopperState(newHopperState));
    }

    void addBranch(String on, String ifTrue, String ifFalse, Conditions condition) {
        armStateMachine.setTransitionCondition(
            new ArmState(movementAdderArmPosition, on), 
            new ArmState(movementAdderArmPosition, ifTrue), 
            condition);
        armStateMachine.setTransitionCondition(
            new ArmState(movementAdderArmPosition, on), 
            new ArmState(movementAdderArmPosition, ifFalse), 
            () -> !condition.check());
    }

    void addOneWayBranch(String on, String ifTrue, Conditions condition) {
        armStateMachine.setTransitionCondition(
            new ArmState(movementAdderArmPosition, on), 
            new ArmState(movementAdderArmPosition, ifTrue), 
            condition);
    }

    //endregion

    void populateHopperIntakeGraph() {
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
        addBranch("branch1 step1", "stop", "branch1 s2", 
            () -> currenForearmState == ForearmState.HopperGrab);

        addForearmMovement("branch1 step2", "branch1 step3", ForearmState.Intermediate);
        addShoulderMovement("branch1 step3", "branch2 step1", ShoulderState.Base2);
        //
    }

    void populateGroundIntakeGraph() {
        setMovementAdderArmPosition(ArmPosition.GroundIntake);

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
        addForearmMovement("branch4 step6", "stop", ForearmState.Ground);
        //

        //branch1
        addBranch("branch1 step1", "stop", "branch1 s2", 
            () -> currenForearmState == ForearmState.HopperGrab);

        addForearmMovement("branch1 step2", "branch1 step3", ForearmState.Intermediate);
        addShoulderMovement("branch1 step3", "branch2 step1", ShoulderState.Base2);
        //
    }
}
