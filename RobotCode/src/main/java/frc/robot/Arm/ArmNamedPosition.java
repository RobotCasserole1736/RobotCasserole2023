package frc.robot.Arm;

import frc.Constants;

public enum ArmNamedPosition {
    //TODO - add actual numbers for the positions
    CUBE_LOW(new ArmEndEffectorState (Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 0.3 ,0.1,false)),
    CUBE_MID(new ArmEndEffectorState (Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 0.58 ,0.6,true), 0.1),
    CUBE_HIGH(new ArmEndEffectorState(Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 1.1 ,1.0,true), 0.1),
    CONE_LOW(new ArmEndEffectorState (Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 0.3,0.1,false)),
    CONE_MID(new ArmEndEffectorState (Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 0.58 ,0.91,false)),
    CONE_HIGH(new ArmEndEffectorState(Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 1.01 ,1.22,true )),
    SHELF(new ArmEndEffectorState    (Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 0.15,1.05,true), 0.05),
    FLOOR(new ArmEndEffectorState    (Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 0.3 ,0.1,false)),
    FLOOR_TIPPED_CONE(new ArmEndEffectorState(Constants.WHEEL_BASE_HALF_LENGTH_M + 0.2,0.10,true), 0.1),
    STOW(new ArmEndEffectorState(0.3, 0.9,false)),
    ;

    public ArmEndEffectorState pos;
    public double safeY;
    
    ArmNamedPosition(ArmEndEffectorState pos){
        this.pos = pos;
        this.safeY = 0.0;
    }

    ArmNamedPosition(ArmEndEffectorState pos, double safeYOffset){
        this.pos = pos;
        this.safeY = pos.y + safeYOffset;
    }
}
