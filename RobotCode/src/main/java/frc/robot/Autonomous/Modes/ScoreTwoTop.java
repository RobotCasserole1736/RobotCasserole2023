package frc.robot.Autonomous.Modes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.AutoSequencer.AutoSequencer;
import frc.lib.Autonomous.AutoMode;
import frc.robot.Arm.ArmNamedPosition;
import frc.robot.Autonomous.Events.AutoEventArmMoveToPos;
import frc.robot.Autonomous.Events.AutoEventDriveAndIntake;
import frc.robot.Autonomous.Events.AutoEventDriveTime;
import frc.robot.Autonomous.Events.AutoEventJSONTrajectory;
import frc.robot.Autonomous.Events.AutoEventSelectConeMode;
import frc.robot.Autonomous.Events.AutoEventSelectCubeMode;
import frc.robot.Autonomous.Events.AutoEventSetClawEject;
import frc.robot.Autonomous.Events.AutoEventSetClawIntake;
import frc.robot.Autonomous.Events.DisableAprilTags;

public class ScoreTwoTop extends AutoMode {

    //This needs to be saved off separately because it's used to supply the default initial pose.
    AutoEventJSONTrajectory initDrive1;
    AutoEventJSONTrajectory initDrive2;

    public ScoreTwoTop(){
        super();
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {
        
        //Place first cone upper
        seq.addEvent(new AutoEventSelectConeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventSetClawEject());

        //Drive to center
        initDrive1 = new AutoEventJSONTrajectory("Score two top pt 1 and score, pickup", 0.45);
        initDrive1.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        seq.addEvent(initDrive1);

        seq.addEvent(new AutoEventSelectCubeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CUBE_LOW));
        seq.addEvent(new AutoEventDriveAndIntake(2.75, 0.50));

        //Drive to grid
        initDrive2 = new AutoEventJSONTrajectory("Score two top pt 2", 0.35);
        initDrive2.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        //initDrive2.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CUBE_HIGH, 5));
        //Does the above line work?!?!?!?!?!?!?!?!?
        seq.addEvent(initDrive2);

        //Place cone upper
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CUBE_HIGH));
        seq.addEvent(new AutoEventSetClawEject());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));


    }

    @Override
    public Pose2d getInitialPose(){
        return initDrive1.getInitialPose();
    }
    
    
}

