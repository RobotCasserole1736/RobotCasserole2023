package frc.robot.Arm;

public class ArmManPosition {

    public void setOpVelCmds(boolean isActive, double x_vel, double y_vel){
        //TODO - save incoming commands
    }

    public void update(){
        //TODO - if we just went from inactive to active, reset the desired position to actual

        //Todo - calcuate the new desired position based on incoming velocity comands
    }

    public ArmEndEffectorPos getCurDesPos(){
        return null; //TODO - return the actual current desired position
    }
    
}