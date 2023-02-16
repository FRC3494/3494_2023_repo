package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Util.ArmState;
import frc.robot.Util.ForearmState;
import frc.robot.Util.StateMachine;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends CommandBase {
	Drivetrain drivetrain;
	Arm arm;
	StateMachine<ArmState> armStateMachine;
	public TeleopDrive(Drivetrain drivetrain, Arm arm) {
		this.drivetrain = drivetrain;
		this.arm = arm;
		addRequirements(drivetrain);
		addRequirements(arm);
	}
	@Override
	public void execute() {
		drivetrain.drive(OI.getTeleopXVelocity(), OI.getTeleopYVelocity(), OI.getTeleopTurnVelocity(), true);
		if(OI.getHopperIntake()){
			gotoHopperIntake();
		}
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(0, 0, 0, false);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
	void pass(){

	}
	//FIX Me: this is a placeholder for when a to position is done
	void stop(){

	}  
	private void gotoHopperIntake(){
		//------------Frome Base 1------------
		if(arm.currentArmState == ArmState.Base1){
			if(arm.currentForearmState == ForearmState.HopperGrab){
				pass();
			}
			else{
				arm.setForearmState(ForearmState.intermediate);
				if(arm.currentForearmState == ForearmState.intermediate){
					arm.setArmState(ArmState.Base2);
					if(arm.currentArmState == ArmState.Base2){
						//the following is copied from Base 2
						arm.setForearmState(ForearmState.HopperGrab);
						if(arm.currentForearmState == ForearmState.HopperGrab){
							arm.setArmState(ArmState.Base1);
							stop();
						}
						
					}
				}
			}
		}
		//------------From Base 2----------------
		else if(arm.currentArmState == ArmState.Base2){
			arm.setForearmState(ForearmState.HopperGrab);
			if(arm.currentForearmState == ForearmState.HopperGrab){
				arm.setArmState(ArmState.Base1);
				stop();
			}
		}
		//----------From Base 3----------
		else if(arm.currentArmState == ArmState.Base4){
			arm.setForearmState(ForearmState.cube2cone1Right);
			if(arm.currentForearmState == ForearmState.cube2cone1Right){
				arm.setArmState(ArmState.Base1);
				if(arm.currentArmState == ArmState.Base1){
					arm.setForearmState(ForearmState.HopperGrab);
					stop();
				}
			}
		}
	}
	//Fixe Me Unfinished 
	private void gotoStore(){
		//-------From Base 1-------------
		if(arm.currentArmState == ArmState.Base1){
			//FIXE ME: Move Hopper to recive
			arm.setForearmState(ForearmState.Store);
			if(arm.currentForearmState == ForearmState.Store){

			}
		}
		//----------Form Basee 2----------
		else if (arm.currentArmState == ArmState.Base2){

		}
		//------From Base 4----------
		else if(arm.currentArmState == ArmState.Base4){

		}
		

	}
}
