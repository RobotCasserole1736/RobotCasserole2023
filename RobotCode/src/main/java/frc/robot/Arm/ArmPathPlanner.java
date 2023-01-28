package frc.robot.Arm;

import edu.wpi.first.wpilibj.Timer;
import frc.Constants;

public class ArmPathPlanner {

    ArmPath curPath = null;
    ArmEndEffectorPos curTargetPos = null;
    ArmEndEffectorPos prevTargetPos = null;
    ArmEndEffectorPos curPositionCmd = null;
    boolean motionActive = false;
    boolean shouldRun = false;
    boolean shouldRunPrev = false;
    double pathStartTime = 0;

    public void setCommand(boolean shouldRun, ArmEndEffectorPos curTargetPos){
        this.shouldRun = shouldRun;
        this.curTargetPos = curTargetPos;
    }

    public void update(ArmEndEffectorPos curPos){
        // calculate if we need a new path
        boolean shouldRunRisingEdge = (shouldRun == true && shouldRunPrev == false);
        boolean targetPosChanged = (curTargetPos != null && prevTargetPos != null && !curTargetPos.isEqualTo(prevTargetPos));
        boolean newPathNeeded = shouldRunRisingEdge || targetPosChanged;

        //If so, make a new path
        if(newPathNeeded){
            curPath = new ArmPath(curPos, curTargetPos, Constants.ARM_END_EFF_MAX_VEL_MPS, Constants.ARM_END_EFF_MAX_ACCEL_MPS2);
            pathStartTime = Timer.getFPGATimestamp();
            motionActive = true;
        }

        var pathTime = Timer.getFPGATimestamp() - pathStartTime;

        if(curPath != null){
            //There is a path init'ed already.
            if(pathTime > curPath.getDurationSec()){
                //Time done - no longer commanding a moving path
                motionActive = false;
            }

            if(motionActive){
                curPositionCmd = curPath.sample(pathTime);
            }
        } else {
            //No path started yet
            motionActive = false;
            curPositionCmd = curPos;
        }

        shouldRunPrev = shouldRun;
        prevTargetPos = curTargetPos;

    }

    public ArmEndEffectorPos getCurDesPos(){
        return curPositionCmd; 
    }
    
}
