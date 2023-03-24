package frc.robot.Arm;


public class ArmAngularState {
    //used for angles of arms
    public double boomAngleDeg;
    public double boomAnglularVel;
    public double boomAnglularAccel;
    public double stickAngleDeg;
    public double stickAngularVel;
    public double stickAngularAccel;

    public ArmAngularState(double boomAngleDeg, double boomAnglularVel, double stickAngleDeg, double stickAngularVel, double boomAnglularAccel, double stickAngularAccel){
        this.boomAngleDeg = boomAngleDeg;
        this.stickAngleDeg = stickAngleDeg;
        this.boomAnglularVel = boomAnglularVel;
        this.stickAngularVel = stickAngularVel;    
        this.boomAnglularAccel = boomAnglularAccel;
        this.stickAngularAccel = stickAngularAccel;    
    }


    public ArmAngularState(double boomAngleDeg, double stickAngleDeg){
        this.boomAngleDeg = boomAngleDeg;
        this.stickAngleDeg = stickAngleDeg;
        this.boomAnglularVel = 0;
        this.stickAngularVel = 0;    
    }

    public ArmAngularState(){
        this.boomAngleDeg = 0;
        this.stickAngleDeg = 0;
        this.boomAnglularVel = 0;
        this.stickAngularVel = 0;    
    }



}