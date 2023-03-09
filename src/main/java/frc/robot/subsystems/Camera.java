package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
    public UsbCamera camera;

    public Timer reconnectTimer;

    public Camera() {
        camera = CameraServer.startAutomaticCapture();

        reconnectTimer = new Timer();
        reconnectTimer.start();
    }

    @Override
    public void periodic() {
        if (!camera.isConnected()
            && reconnectTimer.advanceIfElapsed(1)) {
            camera.close();

            camera = CameraServer.startAutomaticCapture();
        }
    }

    public UsbCamera getCamera() {
        return camera;
    }
}
