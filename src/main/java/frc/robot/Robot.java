package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private Command autonomousCommand;

	private RobotContainer robotContainer;

	@Override
	public void robotInit() {
		robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();

		robotContainer.updateShuffleboardObjects();
	}

	@Override
	public void disabledInit() {
		robotContainer.disabledInit();
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = robotContainer.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

		switch (DriverStation.getAlliance()) {
			case Red:
				OI.setRedOffset();
				break;
			case Blue:
				OI.setBlueOffset();
				break;
			default:
				break;
		}

		robotContainer.teleopInit();
	}

	@Override
	public void teleopPeriodic() {
		OI.update();
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
		OI.update();
	}
}
