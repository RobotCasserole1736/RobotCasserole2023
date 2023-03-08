package frc.robot.Autonomous.Modes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.AutoSequencer.AutoSequencer;
import frc.lib.Autonomous.AutoMode;
import frc.robot.Arm.ArmNamedPosition;
import frc.robot.Autonomous.Events.AutoEventArmMoveToPos;
import frc.robot.Autonomous.Events.AutoEventBraceDrivetrain;
import frc.robot.Autonomous.Events.AutoEventJSONTrajectory;
import frc.robot.Autonomous.Events.AutoEventSelectConeMode;
import frc.robot.Autonomous.Events.AutoEventSetClawEject;

public class ScoreLeaveBalance extends AutoMode {
    
    AutoEventJSONTrajectory initDrive;


    public ScoreLeaveBalance(){
        super();
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {
        
        //Place cone
        seq.addEvent(new AutoEventSelectConeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventSetClawEject());
        
        //Drive to charging station
        initDrive = new AutoEventJSONTrajectory("Score, Leave community, balance", 0.37);
        initDrive.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        seq.addEvent(initDrive);

        //TODO - call auto balancing code here
        seq.addEvent(new AutoEventBraceDrivetrain(10.0));

        
    }

    @Override
    public Pose2d getInitialPose(){
        return initDrive.getInitialPose();
    }

}