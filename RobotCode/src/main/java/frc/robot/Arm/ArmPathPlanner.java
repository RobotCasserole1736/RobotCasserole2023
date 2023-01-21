package frc.robot.Arm;

public class ArmPathPlanner {

    ArmPath curPath;

    //TODO - constructor


    public void setCommand(boolean shouldRun, ArmEndEffectorPos desEndPos){
        //save off commands
    }


    public void update(){
        // calculate if we need a new path

        //If so, make a new path

        //If we're supposed to be running, step through the path

        //Otheriwse, don't output commands.

    }

    public ArmEndEffectorPos getCurDesPos(){
        return null; //TODO actually return the current position of the path
    }
    
}
