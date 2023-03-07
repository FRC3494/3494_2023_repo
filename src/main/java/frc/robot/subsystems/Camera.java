package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;

public class Camera {
    CvSource outputStream;

    public Camera() {
        CameraServer.startAutomaticCapture();
        
        outputStream = CameraServer.putVideo("Video", 320, 240);
    }
}
