package frc.robot.subsystems.camera;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;

import org.bytedeco.depthai.ColorCamera;
import org.bytedeco.depthai.ColorCameraProperties;
import org.bytedeco.depthai.DataOutputQueue;
import org.bytedeco.depthai.Device;
import org.bytedeco.depthai.Pipeline;
import org.bytedeco.depthai.VideoEncoder;
import org.bytedeco.depthai.VideoEncoderProperties;
import org.bytedeco.depthai.XLinkOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Camera extends SubsystemBase {
    Device device;
    private Thread thread;

    public Camera() {
        Pipeline pipeline = createCameraPipeline();

        // Start the pipeline
        device = new Device();

        // Start the pipeline
        device.startPipeline(pipeline);

        CameraServerThread cameraRunnable;
        try {
            cameraRunnable = new CameraServerThread(device);

            thread = new Thread(cameraRunnable);
        } catch (UnknownHostException e) {
            System.out.println("you fucked up the address");
            e.printStackTrace();
        } catch (SocketException e) {
            System.out.println("you fucked up the socket for some reason i don't know how please don't sue me");
            e.printStackTrace();
        }

    }

    public void close() {
        device.close();
        thread.interrupt();
    }

    static Pipeline createCameraPipeline() {
        Pipeline pipeline = new Pipeline();

        ColorCamera colorCam = pipeline.createColorCamera();
        VideoEncoder videEnc = pipeline.createVideoEncoder();
        XLinkOut xout = pipeline.createXLinkOut();

        xout.setStreamName("h265");

        colorCam.setPreviewSize(300, 300);
        colorCam.setResolution(ColorCameraProperties.SensorResolution.THE_1080_P);
        colorCam.setInterleaved(true);

        videEnc.setDefaultProfilePreset(30, VideoEncoderProperties.Profile.H265_MAIN);

        colorCam.preview().link(videEnc.input());
        videEnc.bitstream().link(xout.input());

        return pipeline;
    }
}

class CameraServerThread implements Runnable {
    private Device device;
    private DatagramSocket socket;
    private InetAddress address;
    private ByteBuffer byteBuffer = ByteBuffer.allocate(1_000_000);

    public CameraServerThread(Device device) throws UnknownHostException, SocketException {
        this.address = InetAddress.getByName(Constants.OakDCameraProps.UDP_IP);

        this.device = device;
        this.socket = new DatagramSocket();
    }

    @Override
    public void run() {
        try {
            DataOutputQueue preview = device.getOutputQueue("preview");
            byte[] slice;

            while (true) {

                byteBuffer.put(preview.get().serialize().asByteBuffer());

                if (byteBuffer.limit() > 65000) {
                    slice = byteBuffer.slice(0, 65000).array();
                } else {
                    slice = byteBuffer.slice(0, byteBuffer.limit()).array();
                }

                DatagramPacket packet = new DatagramPacket(slice, slice.length, address,
                        Constants.OakDCameraProps.UDP_PORT);

                socket.send(packet);
            }
        } catch (Exception e) {
            // Throwing an exception
            device.close();
            System.out.println("Exception is caught");
        }
    }
}