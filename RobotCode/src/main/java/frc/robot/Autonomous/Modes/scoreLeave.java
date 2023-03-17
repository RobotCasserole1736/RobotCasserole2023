package frc.robot.Autonomous.Modes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.AutoSequencer.AutoSequencer;
import frc.lib.Autonomous.AutoMode;
import frc.robot.Arm.ArmNamedPosition;
import frc.robot.Autonomous.Events.AutoEventArmMoveToPos;
import frc.robot.Autonomous.Events.AutoEventBraceDrivetrain;
import frc.robot.Autonomous.Events.AutoEventDriveTime;
import frc.robot.Autonomous.Events.AutoEventJSONTrajectory;
import frc.robot.Autonomous.Events.AutoEventSelectConeMode;
import frc.robot.Autonomous.Events.AutoEventSelectCubeMode;
import frc.robot.Autonomous.Events.AutoEventSetClawEject;
import frc.robot.Autonomous.Events.AutoEventSetClawIntake;
import frc.robot.Autonomous.Events.DisableAprilTags;

public class scoreLeave extends AutoMode {

    //This needs to be saved off separately because it's used to supply the default initial pose.
    AutoEventJSONTrajectory initDrive;

    public scoreLeave(){
        super();
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {

        seq.addEvent(new DisableAprilTags());

        seq.addEvent(new AutoEventSelectConeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventSelectCubeMode());

        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));

        seq.addEvent(new AutoEventDriveTime(4, -1));

        seq.addEvent(new AutoEventBraceDrivetrain(4));


    }
    
}

