package frc.robot.Drivetrain;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import frc.Constants;
import frc.lib.Signal.Annotations.Signal;

public class DrivetrainPitchEstimator {

    public enum TiltState{
        LEVEL(0),
        NOSE_UP(1),
        NOSE_DOWN(2);

        public final int value;
        private TiltState(int value) { this.value = value;}
        public int toInt() {return this.value;}
    }

    /* Singleton infrastructure */
    private static DrivetrainPitchEstimator instance;
    public static DrivetrainPitchEstimator getInstance() {
        if (instance == null) {
            instance = new DrivetrainPitchEstimator();
        }
        return instance;
    }

    double TILTED_THRESH_DEG = 10.0;
    double LEVEL_THRESH_DEG = 7.5;

    @Signal
    double chassisPitchDeg = 0;
    @Signal
    double chassisPitchDegRaw = 0;
    BuiltInAccelerometer accel;

    @Signal
    TiltState curTilt = TiltState.LEVEL;



    LinearFilter pitchFilter = LinearFilter.singlePoleIIR(0.1, Constants.Ts);


    private DrivetrainPitchEstimator(){
        accel = new BuiltInAccelerometer(Range.k2G);

    }

    public void update(){

        var vertAccel = accel.getZ();
        var latAccel = accel.getX();

        chassisPitchDegRaw = Units.radiansToDegrees(Math.atan2(vertAccel, latAccel)) - 90;
        

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
