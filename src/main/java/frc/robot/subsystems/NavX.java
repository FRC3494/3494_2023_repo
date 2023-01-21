package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavX {
    private static AHRS ahrs = new AHRS();

    private static double pitchOffset = 0;

    public static void initialize() {
        ahrs.calibrate();
        pitchOffset = ahrs.getRoll();
    }

    public static double getYaw() {
        return ahrs.getFusedHeading() - pitchOffset;
    }

    public static double getPitch() {
        return ahrs.getPitch();
    }

    public static double getRoll() {
        return ahrs.getRoll();
    }

    public static void zeroYaw() {
        pitchOffset = ahrs.getFusedHeading();
    }

    public static void putShuffleBoardData() {
        SmartDashboard.putNumber("NavX offset", pitchOffset);
        SmartDashboard.putNumber("Current Angle", ahrs.getFusedHeading());
    }

    public static void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", NavX::getYaw, null);
    }

    public static AHRS getNavX() {
        return ahrs;
    }
}