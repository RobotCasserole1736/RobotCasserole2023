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

public class ScoreTwoTop extends AutoMode {

    //This needs to be saved off separately because it's used to supply the default initial pose.
    AutoEventJSONTrajectory initDrive;

    public ScoreTwoTop(){
        super();
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {

        //Place first cone upper
        seq.addEvent(new AutoEventSelectConeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventSetClawEject());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));

        //Drive to center
        initDrive = new AutoEventJSONTrajectory("Score two top pt 1", 1.0);
        seq.addEvent(initDrive);

        //Intake cone
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_LOW));
        var drivePickup = new AutoEventDriveTime(2.0, 0.25);
        drivePickup.addChildEvent(new AutoEventSetClawIntake());
        seq.addEvent(drivePickup);
    

        //Drive to grid
        initDrive = new AutoEventJSONTrajectory("Score two top pt 2", 1.0);
        seq.addEvent(initDrive);

        //Place cone upper
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventSetClawEject());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));


    }

    @Override
    public Pose2d getInitialPose(){
        return initDrive.getInitialPose();
    }
    
    
}

