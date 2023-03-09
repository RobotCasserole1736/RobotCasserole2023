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

public class SteakAuto2023 extends AutoMode {

    //This needs to be saved off separately because it's used to supply the default initial pose.
    AutoEventJSONTrajectory initDrive1;
    AutoEventJSONTrajectory initDrive2;
    AutoEventJSONTrajectory initDrive3;

    public SteakAuto2023(){
        super();
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {

        //Place first cone upper
        seq.addEvent(new AutoEventSelectConeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventSelectCubeMode());

        //Drive to center
        initDrive1 = new AutoEventJSONTrajectory("Score two top pt 1 and score, pickup", 0.7);
        initDrive1.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        seq.addEvent(initDrive1);

        //Intake cone
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_LOW));
        seq.addEvent(new AutoEventDriveAndIntake(2.0, 0.4));
        
        //Drive to grid
        initDrive2 = new AutoEventJSONTrajectory("Score two top pt 2", 0.7);
        initDrive2.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        seq.addEvent(initDrive2);

        //Place cone upper
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventSelectCubeMode());
        

        //Drive to charging station
        initDrive3 = new AutoEventJSONTrajectory("Steak balance", 0.5);
        initDrive3.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        seq.addEvent(initDrive3);


    }

    @Override
    public Pose2d getInitialPose(){
        return initDrive1.getInitialPose();
    }
    
    
}

