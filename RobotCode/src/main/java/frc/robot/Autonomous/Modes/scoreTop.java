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

public class scoreTop extends AutoMode {

    //This needs to be saved off separately because it's used to supply the default initial pose.
    AutoEventJSONTrajectory initDrive;

    public scoreTop(){
        super();
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {

        seq.addEvent(new AutoEventSelectConeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventSetClawEject());

        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
    }
    
}

