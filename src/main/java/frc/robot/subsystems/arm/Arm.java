package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.StateMachine;

public class Arm extends SubsystemBase  {
    private DoubleSolenoid basePiston1;
    private DoubleSolenoid basePiston2;

    private CANSparkMax forearmMotor;

	StateMachine<ArmState> armStateMachine;
    ShoulderState currentShoulderState;
    ForearmState currenForearmState;
    
    public Arm(){
        basePiston1 = new DoubleSolenoid(Constants.Subsystems.Arm.BASE_PCM, 
            PneumaticsModuleType.CTREPCM, Constants.Subsystems.Arm.PISTON1_SOLENOID_CHANNEL, 
            Constants.Subsystems.Arm.PISTON1_SOLENOID_CHANNEL + 1);

        basePiston2 = new DoubleSolenoid(Constants.Subsystems.Arm.BASE_PCM, 
            PneumaticsModuleType.CTREPCM, Constants.Subsystems.Arm.PISTON2_SOLENOID_CHANNEL, 
            Constants.Subsystems.Arm.PISTON2_SOLENOID_CHANNEL + 1);

        forearmMotor = new CANSparkMax(Constants.Subsystems.Arm.ARM_MOTOR_CHANNEL, MotorType.kBrushless);

		forearmMotor.getPIDController().setP(Constants.Subsystems.Arm.PIDF.P);
		forearmMotor.getPIDController().setP(Constants.Subsystems.Arm.PIDF.I);
		forearmMotor.getPIDController().setP(Constants.Subsystems.Arm.PIDF.D);
		forearmMotor.getPIDController().setP(Constants.Subsystems.Arm.PIDF.F);
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

    void setForearmState(ForearmState newState){
        forearmMotor.getPIDController().setReference(Constants.Subsystems.Arm.FOREARM_POSITION.get(newState), ControlType.kPosition);
    
        currenForearmState = newState;
    }

    public void setArmState(ArmPosition newState) {
        armStateMachine.transitionTo(new ArmState(newState, "start"));
    }

    void hopperIntakeGraph() {
        armStateMachine.setTransitionCondition(
            new ArmState(ArmPosition.HopperIntake, "start"), 
            new ArmState(ArmPosition.HopperIntake, "branch4"), 
            () -> currentShoulderState == ShoulderState.Base4);
        armStateMachine.setTransitionCondition(
            new ArmState(ArmPosition.HopperIntake, "start"), 
            new ArmState(ArmPosition.HopperIntake, "branch2"), 
            () -> currentShoulderState == ShoulderState.Base2);
        armStateMachine.setTransitionCondition(
            new ArmState(ArmPosition.HopperIntake, "start"), 
            new ArmState(ArmPosition.HopperIntake, "branch1"), 
            () -> currentShoulderState == ShoulderState.Base1);

        //branch4

        //branch2

        //branch1
        armStateMachine.setTransitionCondition(
            new ArmState(ArmPosition.HopperIntake, "branch1"), 
            new ArmState(ArmPosition.HopperIntake, "stop"), 
            () -> currenForearmState == ForearmState.HopperGrab);

        armStateMachine.setTransitionCondition(
            new ArmState(ArmPosition.HopperIntake, "branch1"), 
            new ArmState(ArmPosition.HopperIntake, "branch1step1"), 
            () -> currenForearmState != ForearmState.HopperGrab);
    }
}
