package frc.robot.Drivetrain;

import java.lang.constant.Constable;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import frc.Constants;
import frc.lib.Signal.Annotations.Signal;

public class DrivetrainPitchEstimator {

    /* Singleton infrastructure */
    private static DrivetrainPitchEstimator instance;
    public static DrivetrainPitchEstimator getInstance() {
        if (instance == null) {
            instance = new DrivetrainPitchEstimator();
        }
        return instance;
    }

    @Signal
    double chassisPitchDeg = 0;
    @Signal
    double chassisPitchDegRaw = 0;
    BuiltInAccelerometer accel;

    LinearFilter pitchFilter = LinearFilter.singlePoleIIR(1.0/180.0, Constants.Ts);


    private DrivetrainPitchEstimator(){
        accel = new BuiltInAccelerometer(Range.k2G);

    }

    public void update(){

        var vertAccel = accel.getZ();
        var latAccel = accel.getX();

        chassisPitchDegRaw = Units.radiansToDegrees(Math.atan2(vertAccel, latAccel));

        chassisPitchDeg = pitchFilter.calculate(chassisPitchDegRaw);

    }
    
}
