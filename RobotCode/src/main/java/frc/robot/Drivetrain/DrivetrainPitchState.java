package frc.robot.Drivetrain;

import edu.wpi.first.math.util.Units;
import frc.lib.Signal.Annotations.Signal;

public class DrivetrainPitchState {

    public enum TiltState{
        LEVEL,
        NOSE_UP,
        NOSE_DOWN;
    }

    /* Singleton infrastructure */
    private static DrivetrainPitchState instance;
    public static DrivetrainPitchState getInstance() {
        if (instance == null) {
            instance = new DrivetrainPitchState();
        }
        return instance;
    }

    double TILTED_THRESH_DEG = 10.0;
    double LEVEL_THRESH_DEG = 5.0;

    @Signal
    double chassisPitchDeg = 0;

    TiltState curTilt = TiltState.LEVEL;


    private DrivetrainPitchState(){

    }

    public void update(){

        chassisPitchDeg = Units.radiansToDegrees(DrivetrainPoseEstimator.getInstance().getChassisPitch_rad());

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
