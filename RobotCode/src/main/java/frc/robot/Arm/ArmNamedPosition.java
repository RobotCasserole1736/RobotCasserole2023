package frc.robot.Arm;

public enum ArmNamedPosition {
    //TODO - add actual numbers for the positions
    CUBE_LOW(new ArmEndEffectorState(0.5,0.1,false)),
    CUBE_MID(new ArmEndEffectorState(1.0,0.3,false)),
    CUBE_HIGH(new ArmEndEffectorState(1.5,1.1,true)),
    CONE_LOW(new ArmEndEffectorState(0.5,0.1,false)),
    CONE_MID(new ArmEndEffectorState(1.0,0.3,false)),
    CONE_HIGH(new ArmEndEffectorState(1.5,1.1,true)),
    SHELF(new ArmEndEffectorState(0.2,0.3,true)),
    FLOOR(new ArmEndEffectorState(0.5,0.05,false)),
    FLOOR_TIPPED_CONE(new ArmEndEffectorState(0.5,0.05,true)),
    STOW(new ArmEndEffectorState(0.3, 0.9,false)),
    ;

    ArmEndEffectorState pos;
    
    ArmNamedPosition(ArmEndEffectorState pos){
        this.pos = pos;
    }
}
