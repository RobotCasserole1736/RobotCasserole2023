package frc.robot.Arm;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.ArmTelemetry;
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

    public ArmEndEffectorState update(ArmEndEffectorState curDesState){
        // calculate if we need a new path
        boolean shouldRunRisingEdge = (shouldRun == true && shouldRunPrev == false);
        boolean targetPosChanged = (curTargetPos != null && prevTargetPos != null && !curTargetPos.equals(prevTargetPos) && shouldRun);
        boolean newPathNeeded = shouldRunRisingEdge || targetPosChanged;

        //If so, make a new path
        if(newPathNeeded){
            curPath = ArmPathFactory.build(curDesState, curTargetPos);
            pathStartTime = Timer.getFPGATimestamp();
            ArmTelemetry.getInstance().setDesPath(curPath);
            motionActive = true;
        }

        var pathTime = Timer.getFPGATimestamp() - pathStartTime;

        if(curPath != null && shouldRun){
            curPositionCmd = curPath.sample(pathTime);
        } else {
            //No path started yet
            motionActive = false;
            curPositionCmd = curDesState;
            curPositionCmd.xvel = 0; //ensure we command a "stopped" arm.
            curPositionCmd.yvel = 0;
        }

        shouldRunPrev = shouldRun;
        prevTargetPos = curTargetPos;


        return curPositionCmd;
    }
    
}
