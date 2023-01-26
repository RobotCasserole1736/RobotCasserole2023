package frc.robot.Arm;


public class ArmState {
    //used for angles of arms
    public double boomAngleDeg;
    public double boomAnglularVel;
    public double stickAngleDeg;
    public double stickAngularVel;

    public ArmState(double boomAngleDeg, double boomAnglularVel, double stickAngleDeg, double stickAngularVel){
        this.boomAngleDeg = boomAngleDeg;
        this.stickAngleDeg = stickAngleDeg;
        this.boomAnglularVel = boomAnglularVel;
        this.stickAngularVel = stickAngularVel;    
    }


    public ArmState(double boomAngleDeg, double stickAngleDeg){
        this.boomAngleDeg = boomAngleDeg;
        this.stickAngleDeg = stickAngleDeg;
        this.boomAnglularVel = 0;
        this.stickAngularVel = 0;    
    }

    public ArmState(){
        this.boomAngleDeg = 0;
        this.stickAngleDeg = 0;
        this.boomAnglularVel = 0;
        this.stickAngularVel = 0;    
    }



}