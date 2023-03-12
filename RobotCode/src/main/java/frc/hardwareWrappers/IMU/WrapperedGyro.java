package frc.hardwareWrappers.IMU;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.hardwareWrappers.IMU.ADIS16470.ADIS16470;
import frc.hardwareWrappers.IMU.ADXRS453.RealADXRS453;
import frc.hardwareWrappers.IMU.NavX.RealNavx;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.Robot;

public class WrapperedGyro  {

    AbstractGyro gyro;
    double offset_rad = 0;

    public enum GyroType {
        ADXRS453,
        ADIS16470,
        NAVX
    }

    @Signal(units = "rad")
    private double curAngle_rad;

    public WrapperedGyro(GyroType type){
        if(Robot.isReal()){
            if(type == GyroType.ADXRS453){
                gyro = new RealADXRS453();
            } else if(type == GyroType.ADIS16470){
                gyro = new ADIS16470();
            } else if (type == GyroType.NAVX){
                gyro = new RealNavx();
            }
        } else {
            gyro = new SimGyro();
        }
    }

    public void update(){
        // WPILIB Convention has Gyros inverted in reference frame (positive clockwise)
        // and we maintain our own offset in code when rezeroing.
        curAngle_rad = gyro.getRawYawAngle() * -1.0 + offset_rad;
    }

    /**
     * resets yaw angle to a specified angle, and resets pitch to zero
     * @param curYawAngle_rad
     */
    public void reset(double curYawAngle_rad) {
        offset_rad = curYawAngle_rad;
        gyro.reset();
    }

    public void calibrate() {
        gyro.calibrate();
    }

    public double getYawRate_radpersec() {
        return gyro.getYawRate();
    }

    public double getYawAngle_rad() {
        return curAngle_rad;
    }

    public boolean isConnected() {
        return gyro.isConnected();
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(this.getYawAngle_rad());
    }

    public double getPitch_rad() {
        return gyro.getRawPitchAngle();
    }
    
}
