package frc.robot.Arm;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import frc.Constants;
import frc.lib.Signal.Annotations.Signal;

public class ArmConePlaceOffset {

    @Signal(units="m")
    double curOffsetDist_m = 0.0;

    double prevOffsetDist_m = 0.0;

    @Signal(units="frac")
    double cmdOffsetFrac = 0.0;

    @Signal
    boolean shouldOffset = false;

    SlewRateLimiter posSlewRateLimiter = new SlewRateLimiter(Constants.ARM_END_EFF_MAX_VEL_MPS);

    // Distance to offset the end effector position by.
    final double OFFSET_DIST_M = Units.inchesToMeters(5.0);

    ArmConePlaceOffset(){
        
    }

    public void setCmd(double offsetFrac){
        this.cmdOffsetFrac = offsetFrac;
    }

    public ArmEndEffectorState update(ArmEndEffectorState cmdIn){

        //copy input
        var curPosCmd = new ArmEndEffectorState(cmdIn);

        curOffsetDist_m = posSlewRateLimiter.calculate(this.cmdOffsetFrac * OFFSET_DIST_M);

        curPosCmd.yvel += (curOffsetDist_m - prevOffsetDist_m)/Constants.Ts;
        curPosCmd.y += curOffsetDist_m;

        prevOffsetDist_m = curOffsetDist_m;

        return curPosCmd;
    }
}