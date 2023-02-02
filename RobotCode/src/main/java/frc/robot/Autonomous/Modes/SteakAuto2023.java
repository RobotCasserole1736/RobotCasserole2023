package frc.robot.Autonomous.Modes;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.AutoSequencer.AutoSequencer;
import frc.lib.Autonomous.AutoMode;
import frc.robot.Autonomous.Events.AutoEventJSONTrajectory;

public class SteakAuto2023 extends AutoMode {

    AutoEventJSONTrajectory driveEvent1 = null;
    
    @Override
    public void addStepsToSequencer(AutoSequencer seq) {
        driveEvent1 = new AutoEventJSONTrajectory("steak2023", 0.7);
        seq.addEvent(driveEvent1);
    }

    @Override
    public Pose2d getInitialPose(){
        return driveEvent1.getInitialPose();
        
    }
    
}
