package frc.robot.Arm;

public enum ArmNamedPosition {
    //TODO - add actual numbers for the positions
    CUBE_LOW(1.10 ,0.10,false),
    CUBE_MID(0.9117 ,0.8366,false, 0.1),
    CUBE_HIGH(1.377 ,1.123,false, 0.1),
    CONE_LOW(1.10 ,0.15,false),
    CONE_MID(1.038 ,1.120,false, 0.1),
    CONE_HIGH(1.527 ,1.389,false, 0.1),
    SHELF(1.606 ,1.127,false),
    FLOOR(1.10 ,0.10,false),
    FLOOR_TIPPED_CONE(0.7733 ,0.05,true, 0.1),
    STOW(0.4100, 1.116 ,false),
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
