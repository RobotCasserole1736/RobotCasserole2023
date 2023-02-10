package frc.robot;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;

public class DriverCamera {

    private static DriverCamera inst = null;

    public static synchronized DriverCamera getInstance() {
        if (inst == null)
            inst = new DriverCamera();
        return inst;
    }

    private DriverCamera() {

        // Creates UsbCamera and MjpegServer [1] and connects them
        UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
        usbCamera.setResolution(160,120);
        usbCamera.setPixelFormat(PixelFormat.kMJPEG);
        usbCamera.setExposureAuto();
        MjpegServer mjpegServer1 = new MjpegServer("Driver Camera", 1181);
        mjpegServer1.setSource(usbCamera);
    }

}
