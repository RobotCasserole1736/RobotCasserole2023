package frc.robot.Arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Signal.Annotations.Signal;

public class ArmControl {


    //TODO - define many possible ArmPositions for all the locations we want to place or pickup from
    // These are totally temporary and for test only
    ArmPosition start = new ArmPosition(0.5, 0.5);
    ArmPosition end = new ArmPosition(1.5, 0.75);

    double startTime = Timer.getFPGATimestamp();

    ArmPath path;

    @Signal
    double xPos = 0;

    @Signal
    double yPos = 0;

    //TODO - make a better visualization of actual joint positions, as well as desired end position
    Mechanism2d m2d = new Mechanism2d(2, 2);
    MechanismRoot2d armRoot = m2d.getRoot("Arm", 0, 0);
    MechanismLigament2d armDesPos = armRoot.append(new MechanismLigament2d(null, xPos, startTime));

    public ArmControl(){
        
        SmartDashboard.putData("Arm", m2d);
    }

    //TODO this is jsut for ease of testing, might need to be removed or refactored later
    public void reset(){
        //TODO - don't just regenerate path on reset, do it every time the commanded position changes
        //TODO - use arm's actual position and velocity for the start of the trajectory, not some assumed location
        //TODO - figure out if the path involves transitioning the arm between concave and convex, and slowly transition those angles over time?
        //TODO - maybe here or in a kiunematics class (and related to above) - figure out how we'll handle nice motor ramp rates when the start/end position is the same, but 
        // the joints still need to rotate to transition concave up to concave down. Or something like that
        path = new ArmPath(start, end, 2, 4);
        startTime = Timer.getFPGATimestamp();
    }


    public void update(){
        var desPos = path.sample(Timer.getFPGATimestamp() - startTime);

        //TODO - send des position through kinematics to determine desired joint angles and velocities

        updateTelemetry(desPos.x, desPos.y);
    }

    public void updateTelemetry(double des_x, double des_y){
        //TODO add telemetry for actual joint positions

        //Annoyingly Ligament2d doesn't have a "set endpoint" method so we manually calulcate the distance and angle.
        var trans = new Translation2d(des_x, des_y);
        this.armDesPos.setAngle(trans.getAngle());
        this.armDesPos.setLength(trans.getNorm());
    }
    
}
