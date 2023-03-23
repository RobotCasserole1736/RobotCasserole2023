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
import frc.robot.Autonomous.Events.DisableAprilTags;

public class TestSCurvy extends AutoMode {

    //This needs to be saved off separately because it's used to supply the default initial pose.
    AutoEventJSONTrajectory initDrive;

    public TestSCurvy(){
        super();
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {

        seq.addEvent(new DisableAprilTags());
        
        initDrive = new AutoEventJSONTrajectory("Tune_test_3x3", 0.5);
        seq.addEvent(initDrive);
      
    }

    @Override
    public Pose2d getInitialPose(){
        return initDrive.getInitialPose();
    }
    
    
}

