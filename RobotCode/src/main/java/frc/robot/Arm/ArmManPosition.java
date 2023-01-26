package frc.robot.Arm;

import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;

public class ArmManPosition {

    double des_x_vel;
    double des_y_vel;
    boolean isActive;

    ArmManPosition(){
        
        des_x_vel = 0;
        des_y_vel = 0;
        isActive = false;

    }


    public void setOpVelCmds(boolean isActive, double x_vel, double y_vel){

        des_x_vel = x_vel;
        des_y_vel = y_vel;
        this.isActive = isActive;

    }

    public void update(ArmEndEffectorPos curMeasPos){
        //TODO - if we just went from inactive to active, reset the desired position to actual

        //Todo - calcuate the new desired position based on incoming velocity comands

    }

    public ArmEndEffectorPos getCurDesPos(){
        return null; //TODO - return the actual current desired position
    }

    public boolean cmdActive(){
        return false; //TODO return true when the operator is actively commanding motion
    }
    
}