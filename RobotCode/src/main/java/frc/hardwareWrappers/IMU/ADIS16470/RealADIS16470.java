package frc.hardwareWrappers.IMU.ADIS16470;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import frc.hardwareWrappers.IMU.AbstractGyro;
import frc.lib.Faults.Fault;

public class RealADIS16470 extends AbstractGyro {

    ADIS16470_IMU realGyro;
     
    Fault disconFault = new Fault("Gyro", "Disconnected");

    public RealADIS16470(){
        realGyro = new ADIS16470_IMU();
        disconFault.set(!isConnected());

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
        return Units.degreesToRadians(realGyro.getAngle()) * -1;
    }

    @Override
    public boolean isConnected() {
        return realGyro.isConnected();
    }

    @Override
    public double getPitchRate() {
        return 0; //TODO - apparently the gyro axis is not exposed??
    }

    @Override
    public double getRawPitchAngle() {
        disconFault.set(!isConnected());
        // Multiply by 1.0 since the unit is mounted on the RIO such that positive X axis is 
        // toward the back of the robot.
        return Units.degreesToRadians(realGyro.getXComplementaryAngle()) * -1.0;
    }
    
}
