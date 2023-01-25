package frc.robot.Arm;

import edu.wpi.first.wpilibj.Timer;

public class ArmState {
    //used for angles of arms
    public double boomAngleDeg;
    public double boomAnglularVel;
    public double stickAngleDeg;
    public double stickAngularVel;
    public double obsTime;

    public ArmState( double boomAngleDeg, double stickAngleDeg, ArmState prev){
        commonConstructor(boomAngleDeg, stickAngleDeg, prev);
    }

    public ArmState(){
        commonConstructor(0,0,null);
    }

    public void commonConstructor(double boomAngleDeg, double stickAngleDeg, ArmState prev){
        this.obsTime = Timer.getFPGATimestamp();
        this.boomAngleDeg = boomAngleDeg;
        this.stickAngleDeg = stickAngleDeg;

        if(prev != null){
            var deltaTime = this.obsTime - prev.obsTime;
            this.boomAnglularVel = (this.boomAngleDeg - prev.boomAngleDeg)/deltaTime;
            this.stickAngularVel = (this.stickAngleDeg - prev.stickAngleDeg)/deltaTime;
        } else {
            this.boomAnglularVel = 0;
            this.stickAngularVel = 0;
        }
        
    }



}