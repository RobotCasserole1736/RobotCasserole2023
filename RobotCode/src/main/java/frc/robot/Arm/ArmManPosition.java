package frc.robot.Arm;

import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;

public class ArmManPosition {

    double des_x_vel;
    double des_y_vel;
    boolean isActive;
    boolean isActive_prev; //stores previous value of isActive state
    ArmEndEffectorPos newDesPos;

    ArmManPosition(){
        
        des_x_vel = 0;
        des_y_vel = 0;
        isActive = false;
        isActive_prev = false;
        newDesPos = new ArmEndEffectorPos(0,0,false);
    }


    public void setOpVelCmds(boolean isActive, double x_vel, double y_vel){

        des_x_vel = x_vel;
        des_y_vel = y_vel;

        //isActive_prev copies that state of isActive to compare if it has changed (later on)
        isActive_prev = this.isActive;
        this.isActive = isActive;

    }

    public void update(ArmEndEffectorPos curMeasPos){
        //If we just went from inactive to active, reset the desired position to actual

        if(isActive_prev == false && isActive){
            newDesPos = curMeasPos;
        }

    
        //Calcuate the new desired position based on incoming velocity commands
        newDesPos.x = curMeasPos.x + des_x_vel;
        newDesPos.y = curMeasPos.y + des_y_vel;

    }

    public ArmEndEffectorPos getCurDesPos(){
        return newDesPos;
        
        //Return the actual current desired position
    }

    public boolean cmdActive(){
        return isActive;

        //Return true when the operator is actively commanding motion
    }
}