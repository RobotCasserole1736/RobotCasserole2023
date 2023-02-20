package frc.robot.Autonomous.Modes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.AutoSequencer.AutoSequencer;
import frc.lib.Autonomous.AutoMode;
import frc.robot.Arm.ArmNamedPosition;
import frc.robot.Autonomous.Events.AutoEventArmMoveToPos;
import frc.robot.Autonomous.Events.AutoEventJSONTrajectory;
import frc.robot.Autonomous.Events.AutoEventSelectConeMode;
import frc.robot.Autonomous.Events.AutoEventSetClawEject;

public class ScoreLeaveBalBottom extends AutoMode {
    
    AutoEventJSONTrajectory initDrive;


    public ScoreLeaveBalBottom(){
        super();
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {
        
        //Place cone on high
        seq.addEvent(new AutoEventSelectConeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventSetClawEject());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        
        //Drive to charging station
        initDrive = new AutoEventJSONTrajectory("S, L, B Bot", 1.0);
        seq.addEvent(initDrive);

        //TODO - call auto balancing code here

        
    }

    @Override
    public Pose2d getInitialPose(){
        return initDrive.getInitialPose();
    }

}