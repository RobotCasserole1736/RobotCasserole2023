package frc.robot.Arm;

public enum ArmNamedPosition {
    //TODO - add actual numbers for the positions
    CUBE_LOW(1.10 ,0.15,false),
    CUBE_MID(0.9117 ,0.8366,false, 0.1),
    CUBE_HIGH(1.377 ,1.123,false, 0.1),
    CONE_LOW(1.10 ,0.15,false),
    CONE_MID(1.069 ,1.076,false),
    CONE_HIGH(1.547 ,1.414,false ),
    SHELF(1.069 ,1.076,false),
    FLOOR(1.13 ,0.11,false),
    FLOOR_TIPPED_CONE(1.13 ,0.11,true, 0.1),
    STOW(0.3934, 1.116 ,false),
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
