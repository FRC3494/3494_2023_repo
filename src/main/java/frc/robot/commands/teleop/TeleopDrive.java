package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class TeleopDrive extends CommandBase {
	Drivetrain drivetrain;
	Intake intake;
	//Arm arm;
	//StateMachine<ArmState> armStateMachine;
	public TeleopDrive(Drivetrain drivetrain, Intake intake) {
		this.drivetrain = drivetrain;
		//this.arm = arm;
		addRequirements(drivetrain);
		//addRequirements(arm);
		this.intake = intake;
		addRequirements(intake);
	}
	@Override
	public void execute() {
		drivetrain.drive(OI.getTeleopXVelocity(), OI.getTeleopYVelocity(), OI.getTeleopTurnVelocity(), true);
		intake.setLeftMotorSpeed(OI.getIntakeLeftVelocity());
		intake.setRightMotorSpeed(OI.getIntakeRightVelocity());
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(0, 0, 0, false);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}