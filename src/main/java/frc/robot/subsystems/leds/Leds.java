package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Leds extends SubsystemBase {
    AddressableLED leds;
    AddressableLEDBuffer buffer;

    public Leds() {
        leds = new AddressableLED(Constants.Subsystems.Leds.LED_PORT);
        leds.setLength(Constants.Subsystems.Leds.STRIP_LENGTH);

        buffer = new AddressableLEDBuffer(Constants.Subsystems.Leds.STRIP_LENGTH);

        setPattern(LedPattern.IDLE);
    }

    public void setPattern(LedPattern pattern) {
        switch (pattern) {
            case IDLE:
                for (int i = 0; i < Constants.Subsystems.Leds.STRIP_LENGTH; i++) buffer.setLED(i, Color.kTeal);
            case CUBE:
                for (int i = 0; i < Constants.Subsystems.Leds.STRIP_LENGTH; i++) buffer.setLED(0, Color.kPurple);
            case CONE:
                for (int i = 0; i < Constants.Subsystems.Leds.STRIP_LENGTH; i++) buffer.setLED(0, Color.kYellow);

        }

        leds.setData(buffer);
    }
}
