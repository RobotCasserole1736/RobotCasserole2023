package frc.robot.Arm;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.Constants;

public class ArmManPosition {

    double des_x_vel;
    double des_y_vel;
    boolean isActive;
    boolean isActive_prev; //stores previous value of isActive state
    ArmEndEffectorState newDesPos;

    SlewRateLimiter xVelLimiter;
    SlewRateLimiter yVelLimiter;

    ArmManPosition(){
        
        des_x_vel = 0;
        des_y_vel = 0;
        isActive = false;
        isActive_prev = false;
        xVelLimiter = new SlewRateLimiter(Constants.ARM_END_EFF_MAX_ACCEL_MPS2 * 0.75);
        yVelLimiter = new SlewRateLimiter(Constants.ARM_END_EFF_MAX_ACCEL_MPS2 * 0.75);

        newDesPos = new ArmEndEffectorState(0,0,false);
    }


    public void setOpVelCmds(boolean isActive, double x_vel, double y_vel){

        des_x_vel = x_vel;
        des_y_vel = y_vel;

        //isActive_prev copies that state of isActive to compare if it has changed (later on)
        isActive_prev = this.isActive;
        this.isActive = isActive;

    }

    public ArmEndEffectorState update(ArmEndEffectorState cmdIn){

        //copy input
        var curPosCmd = new ArmEndEffectorState(cmdIn);

        //If we just went from inactive to active, reset the desired position to incoming command
        if(isActive_prev == false && isActive){
            newDesPos = curPosCmd;
        }

        if(isActive){
            //Apply slew rate limit to the velocity commands
            var desXVelLimited = xVelLimiter.calculate(des_x_vel);
            var desYVelLimited = yVelLimiter.calculate(des_y_vel);
            //Calcuate the new desired position based on incoming velocity commands
            newDesPos.xvel = desXVelLimited;
            newDesPos.yvel = desYVelLimited;
            newDesPos.x = curPosCmd.x + desXVelLimited * Constants.Ts;
            newDesPos.y = curPosCmd.y + desYVelLimited * Constants.Ts;
            newDesPos.reflexFrac = curPosCmd.reflexFrac;
            return newDesPos;
        } else {
            return curPosCmd;
        }


    }

    public boolean cmdActive(){
        return isActive;

        //Return true when the operator is actively commanding motion
    }
}