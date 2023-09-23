package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavX {
    private static AHRS ahrs = new AHRS();

    public static void initialize() {
        ahrs.calibrate();
    }

    public static double isCompassValid() {
        return ahrs.getYaw();
    }

    public static double getYaw() {
        return -ahrs.getFusedHeading();
        // return -ahrs.getCompassHeading();
    }

    public static double getPitch() {
        return ahrs.getRoll();
    }

    public static double getRoll() {
        return ahrs.getPitch();
    }

    public static void putShuffleBoardData() {
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