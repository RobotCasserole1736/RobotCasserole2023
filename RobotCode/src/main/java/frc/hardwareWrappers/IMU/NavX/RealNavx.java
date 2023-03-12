package frc.hardwareWrappers.IMU.NavX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.hardwareWrappers.IMU.AbstractGyro;

public class RealNavx extends AbstractGyro {

    AHRS ahrs;

    public RealNavx(){
        ahrs = new AHRS(Port.kMXP);
        this.calibrate();
    }

    @Override
    public void reset() {
        ahrs.reset();
    }

    @Override
    public void calibrate() {
        System.out.println("======================================================");
        System.out.println("== GYRO: CALIBRATION IN PROCESS, DO NOT MOVE ROBOT...");
        ahrs.calibrate();
        System.out.println("== ... Complete!");
        System.out.println("======================================================");
    }

    @Override
    public double getYawRate() {
        return Units.degreesToRadians(ahrs.getRate());
    }

    @Override
    public double getRawYawAngle() {
        return Units.degreesToRadians(ahrs.getAngle());
    }

    @Override
    public boolean isConnected() {
        return ahrs.isConnected();
    }

    @Override
    public double getPitchRate() {
        return 0; //TODO add support here
    }

    @Override
    public double getRawPitchAngle() {
        return 0; //TODO add support here
    }
    
}
