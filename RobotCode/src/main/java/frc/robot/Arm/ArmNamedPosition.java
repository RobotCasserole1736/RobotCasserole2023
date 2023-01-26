package frc.robot.Arm;

public enum ArmNamedPosition {
    //TODO - add actual numbers for the positions
    CUBE_LOW(new ArmEndEffectorPos(0.5,0.1,true)),
    CUBE_MID(new ArmEndEffectorPos(1.0,0.3,true)),
    CUBE_HIGH(new ArmEndEffectorPos(1.5,1.1,true)),
    CONE_LOW(new ArmEndEffectorPos(0.5,0.1,true)),
    CONE_MID(new ArmEndEffectorPos(1.0,0.3,true)),
    CONE_HIGH(new ArmEndEffectorPos(1.5,1.1,true)),
    SHELF(new ArmEndEffectorPos(0.2,0.3,true)),
    FLOOR(new ArmEndEffectorPos(0.5,0.05,false)),
    FLOOR_TIPPED_CONE(new ArmEndEffectorPos(0.5,0.05,true)),
    STOW(new ArmEndEffectorPos(0.1,0.2,false)),
    ;

    ArmEndEffectorPos pos;
    
    ArmNamedPosition(ArmEndEffectorPos pos){
        this.pos = pos;
    }
}
