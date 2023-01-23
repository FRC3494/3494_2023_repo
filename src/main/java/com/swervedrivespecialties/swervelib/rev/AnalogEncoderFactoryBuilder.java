package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;

public class AnalogEncoderFactoryBuilder {
    public AbsoluteEncoderFactory<AnalogEncoderAbsoluteConfiguration> build(CANSparkMax parentSparkMax) {
        return configuration -> {
            SparkMaxAnalogSensor encoder = parentSparkMax.getAnalog(Mode.kAbsolute);
            
            encoder.setPositionConversionFactor(1.0f / 3.3f);
            
            encoder.setInverted(configuration.getReversed());

            return new EncoderImplementation(encoder, configuration.getOffset());
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final SparkMaxAnalogSensor encoder;
        private final double offset;

        private EncoderImplementation(SparkMaxAnalogSensor encoder, double offset) {
            this.encoder = encoder;
            this.offset = offset;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians(encoder.getPosition() * 360.0) - offset;
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }
    }
}
