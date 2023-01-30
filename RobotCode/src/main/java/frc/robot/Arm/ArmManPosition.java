package frc.robot.Arm;

public class ArmManPosition {

    public void setOpVelCmds(boolean isActive, double x_vel, double y_vel){
        //TODO - save incoming commands
    }

    public ArmEndEffectorState update(ArmEndEffectorState curDesPos){
        //TODO - if we just went from inactive to active, reset the desired position to actual

        //Todo - calcuate the new desired position based on incoming velocity comands
        return curDesPos;
    }

    public boolean cmdActive(){
        return false; //TODO return true when the operator is actively commanding motion
    }
    
}
