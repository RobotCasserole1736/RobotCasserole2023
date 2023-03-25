package frc.robot.Autonomous.Modes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.AutoSequencer.AutoSequencer;
import frc.lib.Autonomous.AutoMode;
import frc.robot.Arm.ArmNamedPosition;
import frc.robot.Autonomous.Events.AutoEventArmMoveToPos;
import frc.robot.Autonomous.Events.AutoEventFullPowerYeet;
import frc.robot.Autonomous.Events.AutoEventDriveAndIntake;
import frc.robot.Autonomous.Events.AutoEventDriveTime;
import frc.robot.Autonomous.Events.AutoEventJSONTrajectory;
import frc.robot.Autonomous.Events.AutoEventSelectConeMode;
import frc.robot.Autonomous.Events.AutoEventSelectCubeMode;
import frc.robot.Autonomous.Events.AutoEventSetClawEject;
import frc.robot.Autonomous.Events.AutoEventSetClawIntake;

public class steakYeet2023 extends AutoMode {

    //This needs to be saved off separately because it's used to supply the default initial pose.
    AutoEventJSONTrajectory initDrive1;
    AutoEventJSONTrajectory initDrive2;
    AutoEventJSONTrajectory initDrive3;

    public steakYeet2023(){
        super();
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {

        //Place cone upper
        seq.addEvent(new AutoEventSelectConeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventSelectCubeMode());

        //Drive to center
        initDrive1 = new AutoEventJSONTrajectory("steak pt 1", 0.5);
        initDrive1.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        seq.addEvent(initDrive1);

        //Intake cube
        seq.addEvent(new AutoEventSelectCubeMode());
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CUBE_LOW));
        seq.addEvent(new AutoEventDriveAndIntake(2.00, 0.25));

        //Move to shooting position
        initDrive2 = new AutoEventJSONTrajectory("steak pt 2", 0.17);
        initDrive2.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        seq.addEvent(initDrive2);

        //Shoot cube
        seq.addEvent(new AutoEventArmMoveToPos(ArmNamedPosition.CONE_HIGH));
        seq.addEvent(new AutoEventFullPowerYeet());

        //Drive on charging station
        initDrive3 = new AutoEventJSONTrajectory("Steak pt 3", 0.4);
        initDrive3.addChildEvent(new AutoEventArmMoveToPos(ArmNamedPosition.STOW));
        seq.addEvent(initDrive3);


    }

    @Override
    public Pose2d getInitialPose(){
        return initDrive1.getInitialPose();
    }
    
    
}

