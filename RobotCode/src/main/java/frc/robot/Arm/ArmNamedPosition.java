package frc.robot.Arm;

public enum ArmNamedPosition {
    //TODO - add all possible positions
    SAMPLE_NAME(new ArmEndEffectorPos(0,5));

    ArmEndEffectorPos pos;
    
    ArmNamedPosition(ArmEndEffectorPos pos){
        this.pos = pos;
    }
}
