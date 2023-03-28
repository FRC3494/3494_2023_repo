package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.forearm.Forearm;
import frc.robot.subsystems.wrist.Wrist;

public class TeleopDrive extends CommandBase {
	Drivetrain drivetrain;

    Forearm forearm;
    Wrist wrist;

	public TeleopDrive(Drivetrain drivetrain, Forearm forearm, Wrist wrist) {
		this.drivetrain = drivetrain;
        this.forearm = forearm;
        this.wrist = wrist;

		addRequirements(drivetrain);

        /*
         * 

        OI.forearmFineAdjustPositiveEvent().ifHigh(() -> {
            forearm.directDrive(Constants.OI.FOREARM_FINE_ADJUST_SPEED);
        });

        OI.forearmFineAdjustPositiveEvent().rising().ifHigh(() -> forearm.enableDirectDrive());
        OI.forearmFineAdjustPositiveEvent().falling().ifHigh(() -> {
            forearm.directDrive(0);
            forearm.disableDirectDrive();
        });

        OI.forearmFineAdjustNegativeEvent().ifHigh(() -> {
            forearm.directDrive(-Constants.OI.FOREARM_FINE_ADJUST_SPEED);
        });
        OI.forearmFineAdjustNegativeEvent().rising().ifHigh(() -> forearm.enableDirectDrive());
        OI.forearmFineAdjustNegativeEvent().falling().ifHigh(() -> {
            forearm.directDrive(0);
            forearm.disableDirectDrive();
        });
         */
	}

    @Override
    public void initialize() {
        //forearm.enableDirectDrive();
        //wrist.enableDirectDrive();
    }
	
	@Override
	public void execute() {
		drivetrain.drive(OI.teleopXVelocity(), OI.teleopYVelocity(), -OI.teleopTurnVelocity(), true);

        /*if (Math.abs(OI.forearmFineAdjust()) >= 0.1) {
            forearm.enableDirectDrive();
            //forearm.directDrive(OI.forearmFineAdjust() * Constants.Subsystems.Forearm.);
        } else {
            forearm.disableDirectDrive();
        }*/

        //forearm.directDrive((OI.teleopXVelocity() / 3) * 0.6);
        //wrist.directDrive((OI.teleopTurnVelocity() / 3) * 0.6);
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(0, 0, 0, false);
        //forearm.disableDirectDrive();
        //wrist.disableDirectDrive();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
	
}
