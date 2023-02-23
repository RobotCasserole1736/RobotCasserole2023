package frc.robot.Arm;

import frc.Constants;

public enum ArmNamedPosition {
    //TODO - add actual numbers for the positions
    CUBE_LOW(Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 0.3 ,0.1,false),
    CUBE_MID(Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 0.58 ,0.6,true, 0.1),
    CUBE_HIGH(Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 1.1 ,1.0,true, 0.1),
    CONE_LOW(Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 0.3,0.1,false),
    CONE_MID(Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 0.58 ,0.91,false),
    CONE_HIGH(Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 1.01 ,1.22,true ),
    SHELF(Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 0.15,1.05,true, 0.05),
    FLOOR(Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M + 0.3 ,0.1,false),
    FLOOR_TIPPED_CONE(Constants.WHEEL_BASE_HALF_LENGTH_M + 0.2,0.10,true, 0.1),
    STOW(0.208, 0.668 ,false),
    ;

    public final double safeY;
    public final double posX;
    public final double posY;
    public final boolean isReflex;
    
    ArmNamedPosition(double pos_x, double pos_y, boolean isReflex){
        this.posX = pos_x;
        this.posY = pos_y;
        this.isReflex = isReflex;
        this.safeY = 0.0;
    }

    ArmNamedPosition(double pos_x, double pos_y, boolean isReflex, double safeYOffset){
        this.posX = pos_x;
        this.posY = pos_y;
        this.isReflex = isReflex;
        this.safeY = pos_y + safeYOffset;
    }

    public ArmEndEffectorState get(){
        return new ArmEndEffectorState(this.posX, this.posY, this.isReflex);
    }
}
