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

public class scoreLeavePickBottom extends AutoMode {
    
    AutoEventJSONTrajectory initDrive;

    public scoreLeavePickBottom(){
        super();
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {
        //Place cone on high
        seq.addEvent(new AutoEventSelectConeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventSetClawEject());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        
        //Drive out
        initDrive = new AutoEventJSONTrajectory("S, L Bot", 1.0);
        seq.addEvent(initDrive);

        //pick up
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_LOW));
        var drivePickup = new AutoEventDriveTime(2.0, 0.25);
        drivePickup.addChildEvent(new AutoEventSetClawIntake());
        seq.addEvent(drivePickup);

        //stow
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));

        //TODO - call auto balancing code here

        
    }

    @Override
    public Pose2d getInitialPose(){
        return initDrive.getInitialPose();
    }

}
