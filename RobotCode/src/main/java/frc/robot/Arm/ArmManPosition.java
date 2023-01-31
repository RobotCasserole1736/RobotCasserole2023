package frc.robot.Arm;

import frc.Constants;

public class ArmManPosition {

    double des_x_vel;
    double des_y_vel;
    boolean isActive;
    boolean isActive_prev; //stores previous value of isActive state
    ArmEndEffectorState newDesPos;

    ArmManPosition(){
        
        des_x_vel = 0;
        des_y_vel = 0;
        isActive = false;
        isActive_prev = false;
        newDesPos = new ArmEndEffectorState(0,0,false);
    }


    public void setOpVelCmds(boolean isActive, double x_vel, double y_vel){

        des_x_vel = x_vel;
        des_y_vel = y_vel;

        //isActive_prev copies that state of isActive to compare if it has changed (later on)
        isActive_prev = this.isActive;
        this.isActive = isActive;

    }

    public ArmEndEffectorState update(ArmEndEffectorState curMeasPos){
        //If we just went from inactive to active, reset the desired position to actual

        if(isActive_prev == false && isActive){
            newDesPos = curMeasPos;
        }

    
        //Calcuate the new desired position based on incoming velocity commands
        newDesPos.x = curMeasPos.x + des_x_vel * Constants.Ts;
        newDesPos.y = curMeasPos.y + des_y_vel * Constants.Ts;
        newDesPos.reflexFrac = curMeasPos.reflexFrac;

        return newDesPos;

    }

    public boolean cmdActive(){
        return isActive;

        //Return true when the operator is actively commanding motion
    }
}