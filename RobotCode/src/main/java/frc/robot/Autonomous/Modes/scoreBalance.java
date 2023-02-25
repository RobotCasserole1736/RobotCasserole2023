package frc.robot.Autonomous.Modes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.AutoSequencer.AutoSequencer;
import frc.lib.Autonomous.AutoMode;
import frc.robot.Arm.ArmNamedPosition;
import frc.robot.Autonomous.Events.AutoEventArmMoveToPos;
import frc.robot.Autonomous.Events.AutoEventDriveTime;
import frc.robot.Autonomous.Events.AutoEventJSONTrajectory;
import frc.robot.Autonomous.Events.AutoEventSelectConeMode;
import frc.robot.Autonomous.Events.AutoEventSetClawEject;
import frc.robot.Autonomous.Events.AutoEventSetClawIntake;

public class scoreBalance extends AutoMode {

    //This needs to be saved off separately because it's used to supply the default initial pose.
    AutoEventJSONTrajectory initDrive;

    public scoreBalance(){
        super();
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {
        
        //Place cone
        seq.addEvent(new AutoEventSelectConeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventSetClawEject());
        
        //Drive to charge station
        initDrive = new AutoEventJSONTrajectory("Score, balance", 0.5);
        initDrive.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        seq.addEvent(initDrive);
       
    }

    @Override
    public Pose2d getInitialPose(){
        return initDrive.getInitialPose();
    }
    
    
}

