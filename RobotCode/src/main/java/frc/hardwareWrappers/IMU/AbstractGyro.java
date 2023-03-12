package frc.hardwareWrappers.IMU;


public abstract class AbstractGyro {

    public abstract void reset();
    public abstract void calibrate();
    public abstract double getYawRate();
    public abstract double getRawYawAngle();
    public abstract double getPitchRate();
    public abstract double getRawPitchAngle();
    public abstract boolean isConnected();
    
}
