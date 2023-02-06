package frc.robot.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import frc.lib.Signal.Annotations.Signal;
import frc.lib.Util.MapLookup2D;

/**
 * Wrapper for the logic to do PID control to control a module's azimuth
 * angle
 */
public class AzimuthAngleController{

    // Configure a maximum speed to control to. This is well under what the
    // module could do, but Neo's are overkill for this application.
    final double MAX_AZMTH_SPEED_DEG_PER_SEC = 720.0; 

    PIDController azmthPIDCtrl = new PIDController(0,0,0);

    @Signal(units = "deg")
    double desAng = 0;

    @Signal(units="deg")
    double actAng = 0;

    @Signal(units = "deg")
    double desAngleRateLimit = 0;

    // Ouput command to the motor (volts)
    double azmthMotorCmd = 0;

    // Input from other control logic indicating the translational speed of the chassis.
    double netSpeed = 0;

    @Signal
    boolean invertWheelDirection = false;

    // Table to define a limit on how fast the azimuth motor rotates when the robot
    // is translating quickly. This should help prevent skidding of wheels.
    MapLookup2D azmthCmdLimitTbl;

    double desAnglePrev = -123; //Arbitrary and unlikely starting number. Hacky as all heck but works for now.


    public AzimuthAngleController(){

        azmthPIDCtrl.enableContinuousInput(-180, 180);

        azmthCmdLimitTbl = new MapLookup2D();
        azmthCmdLimitTbl.insertNewPoint(0.0, 1.0);
        azmthCmdLimitTbl.insertNewPoint(1.0, 1.0);
        azmthCmdLimitTbl.insertNewPoint(3.0, 0.5);
        azmthCmdLimitTbl.insertNewPoint(5.0, 0.1);
        azmthCmdLimitTbl.insertNewPoint(9.0, 0.1);

    }

    public void setInputs(double desiredAngle_in, double actualAngle_in, double curSpeed_fps_in){
        
        desAnglePrev = desAng;
        desAng = desiredAngle_in;        
        actAng = actualAngle_in;
        netSpeed = curSpeed_fps_in; 

    }

    public void update(){

        desAngleRateLimit = desAng; //todo - fancy rate limiting... needed?

        azmthMotorCmd = azmthPIDCtrl.calculate(actAng, desAngleRateLimit);
        azmthMotorCmd = limitMag(azmthMotorCmd, azmthCmdLimitTbl.lookupVal(netSpeed));

    }

    public void setGains(double kP, double kI, double kD){
        azmthPIDCtrl.setP(kP);
        azmthPIDCtrl.setI(kI);
        azmthPIDCtrl.setD(kD);
    }

    public double getMotorCmd(){
        return azmthMotorCmd;
    }

    public double getErrMag_deg(){
        return Math.abs(desAng - actAng);
    }

    public double getSetpoint_deg(){
        return desAng;
    }

    private double limitMag(double in, double magMax){
        if(Math.abs(in) > magMax){
            return Math.signum(in) * magMax;
        } else {
            return in;
        }
    }


}