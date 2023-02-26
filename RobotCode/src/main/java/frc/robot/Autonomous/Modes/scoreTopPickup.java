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
import frc.robot.Autonomous.Events.AutoEventWait;

public class scoreTopPickup extends AutoMode {

    //This needs to be saved off separately because it's used to supply the default initial pose.
    AutoEventJSONTrajectory initDrive;

    public scoreTopPickup(){
        super();
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {

        //Place cone 
        seq.addEvent(new AutoEventSelectConeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventSetClawEject());

        //Drive to center
        initDrive = new AutoEventJSONTrajectory("Score two top pt 1 and score, pickup", 1.0);
        initDrive.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        seq.addEvent(initDrive);
 
        //Intake cone
        var drivePickup = new AutoEventDriveTime(2.0, 0.25);
        drivePickup.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_LOW));
        drivePickup.addChildEvent(new AutoEventSetClawIntake());
        seq.addEvent(drivePickup);

        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));



        
   



      
    }

    @Override
    public Pose2d getInitialPose(){
        return initDrive.getInitialPose();
    }
    
    
}

