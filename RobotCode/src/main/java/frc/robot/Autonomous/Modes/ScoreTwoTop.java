package frc.robot.Autonomous.Modes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.AutoSequencer.AutoSequencer;
import frc.lib.Autonomous.AutoMode;
import frc.robot.Arm.ArmNamedPosition;
import frc.robot.Autonomous.Events.AutoEventArmMoveToPos;
import frc.robot.Autonomous.Events.AutoEventClawMiniYeet;
import frc.robot.Autonomous.Events.AutoEventDriveAndIntake;
import frc.robot.Autonomous.Events.AutoEventJSONTrajectory;
import frc.robot.Autonomous.Events.AutoEventSelectConeMode;
import frc.robot.Autonomous.Events.AutoEventSelectCubeMode;
import frc.robot.Autonomous.Events.AutoEventSetClawEject;
import frc.robot.Autonomous.Events.AutoEventWait;
import frc.robot.Autonomous.Events.AutoEventYeet;
import frc.robot.Autonomous.Events.DisableAprilTags;
import frc.robot.Autonomous.Events.EnableAprilTags;

public class ScoreTwoTop extends AutoMode {

    //This needs to be saved off separately because it's used to supply the default initial pose.
    AutoEventJSONTrajectory initDrive1;
    AutoEventJSONTrajectory initDrive2;
    AutoEventJSONTrajectory initDrive3;

    public ScoreTwoTop(){
        super();
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {
        
        seq.addEvent(new DisableAprilTags());

        //Place first cone upper
        seq.addEvent(new AutoEventSelectConeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventSetClawEject());

        //Drive to center
        initDrive1 = new AutoEventJSONTrajectory("Score two top pt 1 and score, pickup Copy", 0.5, 0.05, 0.05);
        initDrive1.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        seq.addEvent(initDrive1);

        /*initDrive2 = new AutoEventJSONTrajectory("Score Two Top 1.5", 0.3, 0.05, 0.05);
        initDrive2.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CUBE_LOW));
        seq.addEvent(initDrive2); */
    
        seq.addEvent(new AutoEventSelectCubeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CUBE_LOW));
        seq.addEvent(new AutoEventDriveAndIntake(2.5, 0.49));

        seq.addEvent(new EnableAprilTags());

        //Drive to grid
        initDrive2 = new AutoEventJSONTrajectory("Score two top pt 2", 0.53);
        initDrive2.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CUBE_MID));
        seq.addEvent(initDrive2);

        //Place cone upper
        //seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CUBE_HIGH));
        //seq.addEvent(new AutoEventWait(.5));
        seq.addEvent(new AutoEventClawMiniYeet());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));


    }

    @Override
    public Pose2d getInitialPose(){
        return initDrive1.getInitialPose();
    }
    
    
}

