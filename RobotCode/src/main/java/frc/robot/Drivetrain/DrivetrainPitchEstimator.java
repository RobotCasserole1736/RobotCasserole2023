package frc.robot.Drivetrain;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import frc.Constants;
import frc.lib.Signal.Annotations.Signal;

public class DrivetrainPitchEstimator {

    public enum TiltState{
        LEVEL,
        NOSE_UP,
        NOSE_DOWN;
    }

    /* Singleton infrastructure */
    private static DrivetrainPitchEstimator instance;
    public static DrivetrainPitchEstimator getInstance() {
        if (instance == null) {
            instance = new DrivetrainPitchEstimator();
        }
        return instance;
    }

    double TILTED_THRESH_DEG = 20.0;
    double LEVEL_THRESH_DEG = 10.0;

    @Signal
    double chassisPitchDeg = 0;
    @Signal
    double chassisPitchDegRaw = 0;
    BuiltInAccelerometer accel;

    TiltState curTilt = TiltState.LEVEL;



    LinearFilter pitchFilter = LinearFilter.singlePoleIIR(1.0/180.0, Constants.Ts);


    private DrivetrainPitchEstimator(){
        accel = new BuiltInAccelerometer(Range.k2G);

    }

    public void update(){

        var vertAccel = accel.getZ();
        var latAccel = accel.getX();

        chassisPitchDegRaw = Units.radiansToDegrees(Math.atan2(vertAccel, latAccel));

        chassisPitchDeg = pitchFilter.calculate(chassisPitchDegRaw);

        //Interpret the pitch into a three-state level, nose-up, or nose-down state with hystersis
        if(Math.abs(chassisPitchDeg) < LEVEL_THRESH_DEG){
            curTilt = TiltState.LEVEL;
        } else if(chassisPitchDeg > TILTED_THRESH_DEG){
            curTilt = TiltState.NOSE_UP;
        } else if(chassisPitchDeg < -1.0 * TILTED_THRESH_DEG){
            curTilt = TiltState.NOSE_DOWN;
        } //else hold state

       
    }

    public double getPitchDeg(){
        return chassisPitchDeg;
    }

    public TiltState getCurTilt(){
        return curTilt;
    }
    
}
