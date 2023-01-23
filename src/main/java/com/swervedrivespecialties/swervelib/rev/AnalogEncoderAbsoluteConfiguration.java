package com.swervedrivespecialties.swervelib.rev;

public class AnalogEncoderAbsoluteConfiguration {
    private final int id;
    private final boolean reverse;
    private final double offset;

    public AnalogEncoderAbsoluteConfiguration(int id, boolean reverse, double offset) {
        this.id = id;
        this.reverse = reverse;
        this.offset = offset;
    }

    public int getId() {
        return id;
    }

    public boolean getReversed() {
        return reverse;
    }

    public double getOffset() {
        return offset;
    }
}
