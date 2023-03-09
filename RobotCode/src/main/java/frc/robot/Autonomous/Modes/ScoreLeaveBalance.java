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
    
    AutoEventJSONTrajectory initDrive1;
    AutoEventJSONTrajectory initDrive2;



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
        initDrive1 = new AutoEventJSONTrajectory("Score, Leave community, balance pt 1", 0.25);
        initDrive1.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        seq.addEvent(initDrive1);

        initDrive2 = new AutoEventJSONTrajectory("Score, Leave community, balance pt 2", 0.45);
        seq.addEvent(initDrive2);




        //TODO - call auto balancing code here
        seq.addEvent(new AutoEventBraceDrivetrain(10.0));

        
    }

    @Override
    public Pose2d getInitialPose(){
        return initDrive1.getInitialPose();
    }

}