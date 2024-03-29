package frc.robot.Arm;

public enum ArmNamedPosition {
    CUBE_LOW(1.10 ,0.28,false),
    CUBE_MID(0.8788 ,0.8886,false, 0.1),
    CUBE_HIGH(1.276 ,1.149,false, 0.1),
    // cube_mid is the right height for yeeting the cube to the high goal 
    CONE_LOW(1.084 ,0.2082,false),
    CONE_MID(1.036 ,1.102,false, 0.1),
    CONE_HIGH(1.502 ,1.399,false, 0.15),
    SHELF(1.55 ,1.25,true),
    FLOOR(1.10 ,0.28,false),
    FLOOR_TIPPED_CONE(0.6539 ,0.07,true),
    STOW(0.2002, 1.127 ,false),
    CUBE_SHOOT_STATION(1.38, 1.343, false),
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
