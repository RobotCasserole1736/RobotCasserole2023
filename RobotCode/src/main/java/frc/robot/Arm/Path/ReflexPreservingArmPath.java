package frc.robot.Arm.Path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import frc.Constants;
import frc.robot.Arm.ArmEndEffectorState;
import frc.robot.Arm.ArmNamedPosition;

/**
 * Thin wrapper on wpilib's trajectory to re-orient the reference frame from
 * a field into our arm's plane. ReflexPreserving implies the arm will retain
 * the same reflex throughout, and does not need to go to a fully-extended position
 * to switch reflex.
 */
public class ReflexPreservingArmPath implements ArmPath {


    //WPILib Trajectory
    Trajectory traj;

    ArmEndEffectorState start; 
    List<Translation2d> interiorWaypoints;
    ArmNamedPosition end;
    double max_vel;
    double max_accel; 
    double totalDuration = 0;

    /**
     * Create a new arm trajectory from start to end with the given constraints
     * @param start
     * @param end
     * @param max_vel_mps
     * @param max_accel_mps2

     */
    @Override
    public void build(ArmEndEffectorState start, ArmNamedPosition end, double max_vel_mps, double max_accel_mps2){
        this.start = start; 
        this.end = end;

        this.max_vel = max_vel_mps;
        this.max_accel = max_accel_mps2; 

        TrajectoryConfig cfg = new TrajectoryConfig(max_vel_mps, max_accel_mps2);
        cfg.addConstraint(new CentripetalAccelerationConstraint(max_accel_mps2));

        interiorWaypoints = new ArrayList<Translation2d>();//none by default

        Pose2d pathStartPos;
        if(start.x > Constants.WHEEL_BASE_HALF_LENGTH_M){
            // If we're outside frame perimiter, ensure we start with upward motion
            pathStartPos = start.toPoseToTop();
        } else {
            pathStartPos = start.toPoseToOther(end);
        }

        Pose2d pathEndPos;

        if(end.safeY > 0){
            //If the end has a configured safe height, add in an additional waypoint to account for it
            // TODO - handle safe Y better
            pathEndPos = end.pos.toPoseFromTop();
        } else {
            // If we end outside the frame perimiter, approach from top.
            // Otherwise, just go straight to it.
            if(end.pos.x > Constants.WHEEL_BASE_HALF_LENGTH_M){
                pathEndPos = end.pos.toPoseFromTop();
            } else {
                pathEndPos = end.pos.toPoseFromOther(start);
            }
        }



        if(end.pos.distTo(start) > MIN_PATHPLAN_DIST_M) {
            traj = TrajectoryGenerator.generateTrajectory(pathStartPos, interiorWaypoints, pathEndPos, cfg);
            totalDuration = traj.getTotalTimeSeconds();
        } else {
            //Error while planning. Just give up.
            totalDuration = 0.0;
            DriverStation.reportWarning("Trajectory too small!", false);
        }

    }

    /**
     * Return the ArmPosition at a given time
     * @param time_sec
     * @return
     */
    public ArmEndEffectorState sample(double time_sec){
        if(time_sec < totalDuration){
            double curReflex = end.pos.reflexFrac;
            return ArmEndEffectorState.fromTrajState(traj, time_sec, curReflex);
        } else {
            return end.pos;
        }

    }

    public double getDurationSec(){
        return totalDuration;
    }

    @Override
    public List<Translation2d> getWaypoints() {
        var retList = new ArrayList<Translation2d>();
        retList.add(new Translation2d(start.x, start.y));
        retList.addAll(interiorWaypoints);
        retList.add(new Translation2d(end.pos.x, end.pos.y));
        return retList;
    }

}
