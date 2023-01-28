package frc.robot.Arm;

public class ArmManPosition {

    public void setOpVelCmds(boolean isActive, double x_vel, double y_vel){
        //TODO - save incoming commands
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
