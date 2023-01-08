package com.swervedrivespecialties.swervelib.rev;

public class SparkMAXEncoderAbsoluteConfiguration {
    private final int id;
    private final boolean reverse;

    public SparkMAXEncoderAbsoluteConfiguration(int id, boolean reverse) {
        this.id = id;
        this.reverse = reverse;
    }

    public int getId() {
        return id;
    }

    public boolean getReversed() {
        return reverse;
    }
}
