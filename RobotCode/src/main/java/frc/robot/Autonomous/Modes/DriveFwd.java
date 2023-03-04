package frc.robot.Autonomous.Modes;

import frc.lib.AutoSequencer.AutoSequencer;
import frc.lib.Autonomous.AutoMode;
import frc.robot.Autonomous.Events.AutoEventDriveTime;

public class DriveFwd extends AutoMode {

    private double duration = 0;

    public DriveFwd(double duration){
        super();
        this.duration = duration;
        this.humanReadableName = "Drive Fwd " + Double.toString(duration) + "s";
    }

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {
        seq.addEvent(new AutoEventDriveTime(duration, 1.35));
    }
    
}

