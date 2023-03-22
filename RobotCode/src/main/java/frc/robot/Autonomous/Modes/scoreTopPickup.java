package frc.robot.Autonomous.Modes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.AutoSequencer.AutoSequencer;
import frc.lib.Autonomous.AutoMode;
import frc.robot.Arm.ArmNamedPosition;
import frc.robot.Autonomous.Events.AutoEventArmMoveToPos;
import frc.robot.Autonomous.Events.AutoEventDriveAndIntake;
import frc.robot.Autonomous.Events.AutoEventJSONTrajectory;
import frc.robot.Autonomous.Events.AutoEventSelectConeMode;
import frc.robot.Autonomous.Events.AutoEventSetClawEject;

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
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));

        //Drive to center
        initDrive = new AutoEventJSONTrajectory("Score two top pt 1 and score, pickup", 0.75);
        seq.addEvent(initDrive);
 
        //Intake cone
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_LOW));
        seq.addEvent(new AutoEventDriveAndIntake(2.0, 0.25));
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        
      
    }

    @Override
    public Pose2d getInitialPose(){
        return initDrive.getInitialPose();
    }
    
    
}

