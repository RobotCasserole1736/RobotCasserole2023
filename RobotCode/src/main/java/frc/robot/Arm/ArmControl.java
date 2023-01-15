package frc.robot.Arm;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.Signal.Annotations.Signal;

public class ArmControl {

    //Completely temproary for testing only

    ArmPosition start = new ArmPosition(3, 3);
    ArmPosition end = new ArmPosition(3, 33);
    ArmPath path = new ArmPath(start, end, 2, 4, 8);

    double startTime = Timer.getFPGATimestamp();

    @Signal
    double curDist = 0;

    public void reset(){
        startTime = Timer.getFPGATimestamp();
    }

    public void update(){
        var dist = path.calculate(Timer.getFPGATimestamp() - startTime);
        this.curDist = dist.y;
    }
    
}
