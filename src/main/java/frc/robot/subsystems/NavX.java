package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavX {
    private static AHRS ahrs = new AHRS();

    public static void initialize() {
        ahrs.calibrate();
    }

    public static double getYaw() {
        return ahrs.getFusedHeading();
    }

    public static double getPitch() {
        
        return ahrs.getPitch();
    }

    public static double getRoll() {
        return ahrs.getRoll();
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