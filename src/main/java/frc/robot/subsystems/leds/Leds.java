package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Leds extends SubsystemBase {
    AddressableLED leds;
    AddressableLEDBuffer buffer;

    LedPattern currentPattern = LedPattern.IDLE;

    public Leds() {
        leds = new AddressableLED(Constants.Subsystems.Leds.LED_PORT);
        leds.setLength(Constants.Subsystems.Leds.STRIP_LENGTH);

        buffer = new AddressableLEDBuffer(Constants.Subsystems.Leds.STRIP_LENGTH);

        leds.start();
    }

    public void setPattern(LedPattern pattern) {
        currentPattern = pattern;
    }

    public LedPattern getPattern(LedPattern pattern) {
        return currentPattern;
    }

    @Override
    public void periodic() {
        switch (currentPattern) {
            case IDLE:
                for (int i = 0; i < buffer.getLength(); i++) buffer.setLED(i, Color.kTeal);
                break;
            case CUBE:
                for (int i = 0; i < buffer.getLength(); i++) buffer.setLED(i, Color.kPurple);
                break;
            case CONE:
                for (int i = 0; i < buffer.getLength(); i++) buffer.setLED(i, Color.kYellow);
                break;
        }

        leds.setData(buffer);
    }
}
