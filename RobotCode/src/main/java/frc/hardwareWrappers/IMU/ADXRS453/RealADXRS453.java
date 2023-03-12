package frc.hardwareWrappers.IMU.ADXRS453;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.hardwareWrappers.IMU.AbstractGyro;
import frc.lib.Faults.Fault;

public class RealADXRS453 extends AbstractGyro {

    ADXRS450_Gyro realGyro;

    Fault disconFault = new Fault("Gyro", "Disconnected");

    public RealADXRS453(){
        realGyro = new ADXRS450_Gyro(Port.kOnboardCS0);
        disconFault.set(!isConnected());
        realGyro.calibrate();
    }

    @Override
    public void reset() {
        realGyro.reset();
    }

    @Override
    public void calibrate() {
        System.out.println("======================================================");
        System.out.println("== GYRO: CALIBRATION IN PROCESS, DO NOT MOVE ROBOT...");
        realGyro.calibrate();
        System.out.println("== ... Complete!");
        System.out.println("======================================================");
    }

    @Override
    public double getYawRate() {
        disconFault.set(!isConnected());
        return Units.degreesToRadians(realGyro.getRate());
    }

    @Override
    public double getRawYawAngle() {
        disconFault.set(!isConnected());
        return Units.degreesToRadians(realGyro.getAngle());
    }

    @Override
    public boolean isConnected() {
        return realGyro.isConnected();
    }

    @Override
    public double getPitchRate() {
        return 0; //unsupported
    }

    @Override
    public double getRawPitchAngle() {
        return 0; //unsupported
    }
    
}
