package frc.robot.Arm;

import edu.wpi.first.wpilibj.Timer;
import frc.Constants;
import frc.robot.Arm.Path.ArmPath;
import frc.robot.Arm.Path.ArmPathFactory;

public class ArmPathPlanner {

    ArmPath curPath = null;
    ArmNamedPosition curTargetPos = null;
    ArmNamedPosition prevTargetPos = null;
    ArmEndEffectorState curPositionCmd = null;
    boolean motionActive = false;
    boolean shouldRun = false;
    boolean shouldRunPrev = false;
    double pathStartTime = 0;

    public void setCommand(boolean shouldRun, ArmNamedPosition curTargetPos){
        this.shouldRun = shouldRun;
        this.curTargetPos = curTargetPos;
    }

    public void update(ArmEndEffectorState curPos){
        // calculate if we need a new path
        boolean shouldRunRisingEdge = (shouldRun == true && shouldRunPrev == false);
        boolean targetPosChanged = (curTargetPos != null && prevTargetPos != null && !curTargetPos.equals(prevTargetPos));
        boolean newPathNeeded = shouldRunRisingEdge || targetPosChanged;

        //If so, make a new path
        if(newPathNeeded){
            curPath = ArmPathFactory.build(curPos, curTargetPos);
            pathStartTime = Timer.getFPGATimestamp();
            motionActive = true;
        }

        var pathTime = Timer.getFPGATimestamp() - pathStartTime;

        if(curPath != null && shouldRun){
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

    public ArmEndEffectorState getCurDesPos(){
        return curPositionCmd; 
    }
    
}
