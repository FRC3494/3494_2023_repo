package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;

public class SparkMAXEncoderFactoryBuilder {
    public AbsoluteEncoderFactory<SparkMAXEncoderAbsoluteConfiguration> build(CANSparkMax parentSparkMax) {
        return configuration -> {
            return new EncoderImplementation(parentSparkMax.getAbsoluteEncoder(Type.kDutyCycle));
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final SparkMaxAbsoluteEncoder encoder;

        private EncoderImplementation(SparkMaxAbsoluteEncoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians(encoder.getPosition());
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }
    }
}
