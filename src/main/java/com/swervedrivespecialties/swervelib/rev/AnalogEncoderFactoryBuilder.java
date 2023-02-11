package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class AnalogEncoderFactoryBuilder {
    public AbsoluteEncoderFactory<AnalogEncoderAbsoluteConfiguration> build(CANSparkMax parentSparkMax) {
        return configuration -> {
            AnalogPotentiometer encoder = new AnalogPotentiometer(new AnalogInput(configuration.getId()), 360);

            return new EncoderImplementation(encoder, configuration.getOffset(), configuration.getReversed());
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final AnalogPotentiometer encoder;
        private final double offset;
        private final boolean inverted;

        private EncoderImplementation(AnalogPotentiometer encoder, double offset, boolean inverted) {
            this.encoder = encoder;
            this.offset = offset;
            this.inverted = inverted;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians(encoder.get() * (inverted ? -1 : 1)) - offset;
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }
    }
}
