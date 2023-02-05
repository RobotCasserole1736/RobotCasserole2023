package frc.robot.Arm;

import edu.wpi.first.math.util.Units;
import frc.Constants;
import frc.lib.Signal.Annotations.Signal;

public class ArmConePlaceOffset {

    @Signal(units="m")
    double curOffsetDist_m = 0.0;

    @Signal
    boolean shouldOffset = false;

    // Velocity to move the arm at when doing an offset
    final double OFFSET_VEL_MPS = Units.inchesToMeters(5.0);

    // Distance to offset the end effector position by.
    final double OFFSET_DIST_M = -1.0 * Units.inchesToMeters(5.0);

    ArmConePlaceOffset(){
        
    }

    public void setCmd(boolean shouldOffset){
        this.shouldOffset = shouldOffset;
    }

    public ArmEndEffectorState update(ArmEndEffectorState cmdIn){

        //copy input
        var curPosCmd = new ArmEndEffectorState(cmdIn);

        double vel = 0.0;

        if(this.shouldOffset){
            if(curOffsetDist_m > OFFSET_DIST_M){
                //Offset needed, but not there yet. Move to offset position.
                vel = OFFSET_VEL_MPS * -1.0;
                curOffsetDist_m += vel * Constants.Ts;
            } else {
                // At offset position
                vel = 0;
                curOffsetDist_m = OFFSET_DIST_M;
            }
        } else {
            if(curOffsetDist_m < 0.0){
                //Offset not needed, but not fully undone. Move to nominal position.
                vel = OFFSET_VEL_MPS;
                curOffsetDist_m += vel * Constants.Ts;
            } else {
                // At nominal position
                vel = 0.0;
                curOffsetDist_m = 0.0;
            }
        }

        curPosCmd.yvel += vel;
        curPosCmd.y += curOffsetDist_m;

        return curPosCmd;
    }
}