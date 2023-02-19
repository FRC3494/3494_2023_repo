package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
    Compressor compressor;

    public Pneumatics() {
        compressor = new Compressor(Constants.Subsystems.Pneumatics.BASE_PCM, PneumaticsModuleType.REVPH);
    }

    public void enable() {
        compressor.enableAnalog(Constants.Subsystems.Pneumatics.MIN_PRESSURE, Constants.Subsystems.Pneumatics.MAX_PRESSURE);
    }

    public void disable() {
        compressor.disable();
    }
}
