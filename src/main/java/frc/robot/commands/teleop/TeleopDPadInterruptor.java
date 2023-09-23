package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;

public class TeleopDPadInterruptor extends CommandBase {
    @Override
    public boolean isFinished() {
        return OI.isDPadPressed();
    }
}
