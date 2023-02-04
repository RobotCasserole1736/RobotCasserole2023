package frc.robot.Arm.Path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Arm.ArmEndEffectorState;
import frc.robot.Arm.ArmNamedPosition;

/**
 * Thin wrapper on wpilib's trajectory to re-orient the reference frame from
 * a field into our arm's plane. ReflexPreserving implies the arm will retain
 * the same reflex throughout, and does not need to go to a fully-extended position
 * to switch reflex.
 */
public class LinearInterpolatedArmPath implements ArmPath {


    ArmEndEffectorState start; 
    ArmEndEffectorState end;
    double linearVel = 0;
    double totalDuration = 0;

    /**
     * Create a new arm trajectory from start to end with the given constraints
     * @param start
     * @param end
     * @param max_vel_mps
     * @param max_accel_mps2

     */
    @Override
    public boolean build(ArmEndEffectorState start, ArmNamedPosition end, double max_vel_mps, double max_accel_mps2){
        this.start = start; 
        this.end = end.get();
        this.linearVel = max_vel_mps;

        totalDuration = start.distTo(this.end) / max_vel_mps;

        return true;
    }

    /**
     * Return the ArmPosition at a given time
     * @param time_sec
     * @return
     */
    public ArmEndEffectorState sample(double time_sec){
        if(time_sec < totalDuration){
            var frac = (time_sec)/(totalDuration);
            return start.interpolateTo(end, frac, this.linearVel);
        } else {
            return end;
        }

    }

    public double getDurationSec(){
        return totalDuration;
    }

    @Override
    public List<Translation2d> getWaypoints() {
        var retList = new ArrayList<Translation2d>();
        retList.add(new Translation2d(start.x, start.y));
        retList.add(new Translation2d(end.x, end.y));
        return retList;
    }

}
