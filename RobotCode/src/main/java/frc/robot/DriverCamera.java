package frc.robot;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import frc.lib.Faults.Fault;

public class DriverCamera {

    private static DriverCamera inst = null;

    Fault disconnFault = new Fault("Driver Camera", "Disconnected");

    UsbCamera usbCamera = null;

    public static synchronized DriverCamera getInstance() {
        if (inst == null)
            inst = new DriverCamera();
        return inst;
    }

    private DriverCamera() {


        MjpegServer mjpegServer1 = new MjpegServer("Driver Camera", 1181);

        try { 
            usbCamera = new UsbCamera("USB Camera 0", 0);
            usbCamera.setResolution(160,120);
            usbCamera.setPixelFormat(PixelFormat.kMJPEG);
            usbCamera.setExposureAuto();
            mjpegServer1.setSource(usbCamera);
        } catch(VideoException e) {
            usbCamera = null;
        }

    }

    public void update(){
        if(usbCamera != null){
            disconnFault.set(!usbCamera.isConnected());
        } else {
            disconnFault.reportFault();
        }
    }

}
