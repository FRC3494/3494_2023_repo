package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
    DoubleSolenoid clawSolenoid;

    boolean currentlyClosed = false;

	public Claw() {
        clawSolenoid = new DoubleSolenoid(Constants.Subsystems.Pneumatics.BASE_PCM, PneumaticsModuleType.REVPH, Constants.Subsystems.Claw.CLAW_SOLENOID_CHANNEL, Constants.Subsystems.Claw.CLAW_SOLENOID_CHANNEL + 1);
	
        set(false);
    }
    
    public void set(boolean closed) {
        clawSolenoid.set(closed ? Value.kForward : Value.kReverse);

        currentlyClosed = closed;
    }

    public void toggle() {
        set(!currentlyClosed);
    }
}
