package frc.hardwareWrappers.Gyro.ADXRS453;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.hardwareWrappers.Gyro.AbstractGyro;
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
    public double getRate() {
        disconFault.set(isConnected());
        return Units.degreesToRadians(realGyro.getRate());
    }

    @Override
    public double getRawAngle() {
        disconFault.set(isConnected());
        return Units.degreesToRadians(realGyro.getAngle());
    }

    @Override
    public boolean isConnected() {
        return realGyro.isConnected();
    }
    
}
