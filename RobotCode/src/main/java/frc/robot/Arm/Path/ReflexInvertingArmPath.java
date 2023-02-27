package frc.robot.Arm.Path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.Constants;
import frc.robot.Arm.ArmEndEffectorState;
import frc.robot.Arm.ArmNamedPosition;

/**
 * Thin wrapper on wpilib's trajectory to re-orient the reference frame from
 * a field into our arm's plane. Reflex-Inverting implies the arm will fully extend
 * partway through the path, flip its reflex, then continue onward. The point of
 * full extension is called the "reflex point", and is where we can safely flip
 * the reflex from one to the other without causing jerky motion of the arm.
 */
public class ReflexInvertingArmPath implements ArmPath {


    //WPILib Trajectory
    Trajectory traj1;
    Trajectory traj2;

    ArmEndEffectorState start; 
    List<Translation2d> interiorWaypoints1;
    List<Translation2d> interiorWaypoints2;
    ArmNamedPosition end;
    double max_vel;
    double max_accel; 
    double duration1;
    double totalDuration;

    // All these constants are related to finding two points:
    // reflexEnd - the reflex point of "full extension" at some angle upward from the horizon
    final double reflexDist = (Constants.ARM_STICK_LENGTH + Constants.ARM_BOOM_LENGTH);
    final double reflexAngleRad = Units.degreesToRadians(20.0);
    final double reflexMidPointFrac = 0.90;
    final double reflexEndPosX = reflexDist * Math.cos(reflexAngleRad);
    final double reflexEndPosY = reflexDist * Math.sin(reflexAngleRad) + Constants.ARM_BOOM_MOUNT_HIEGHT;

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
        this.end = end;

        this.max_vel = max_vel_mps;
        this.max_accel = max_accel_mps2; 

        TrajectoryConfig cfg = new TrajectoryConfig(max_vel_mps, max_accel_mps2);
        cfg.addConstraint(new CentripetalAccelerationConstraint(max_accel_mps2));

        var reflexEndpoint = new ArmEndEffectorState(reflexEndPosX, reflexEndPosY);

        ///////////////////////////////////////////////////////
        // First Trajectory - start out to the reflex point

        interiorWaypoints1 = new ArrayList<Translation2d>();

        Pose2d pathStartPos;
        if(start.x > Constants.WHEEL_BASE_HALF_LENGTH_M && Math.abs(start.x - end.get().x) > 0.01){
            // If we're outside frame perimiter and about to move horizontally, 
            // ensure we start with upward motion to clear obstacles
            pathStartPos = start.toPoseToTop();
        } else {
            pathStartPos = start.toPoseToOther(end);
        }

        var failed = true;
        try {
            traj1 = TrajectoryGenerator.generateTrajectory(pathStartPos, interiorWaypoints1, reflexEndpoint.toPoseFromOther(start), cfg);
            duration1 = traj1.getTotalTimeSeconds();
            failed = false;
        } catch (Exception e){
        }

        if(failed){
            duration1 = 0;
            DriverStation.reportWarning("Trajectory 1 gen failed", false);
        }
        

        totalDuration += duration1;

        ///////////////////////////////////////////////////////
        // Second Trajectory - reflex point to the end position
        interiorWaypoints2 = new ArrayList<Translation2d>();//Reset to be blank for part two

        Pose2d pathEndPos;

        if(end.safeY > 0){
            //If the end has a configured safe height, add in an additional waypoint to account for it
            // TODO - handle safe Y better
            pathEndPos = end.get().toPoseFromTop();
        } else {
            // If we end outside the frame perimiter, approach from top.
            // Otherwise, just go straight to it.
            if(end.get().x > Constants.WHEEL_BASE_HALF_LENGTH_M){
                pathEndPos = end.get().toPoseFromTop();
            } else {
                pathEndPos = end.get().toPoseFromOther(start);
            }
        }



        try {
            traj2 = TrajectoryGenerator.generateTrajectory(reflexEndpoint.toPoseToOther(end), interiorWaypoints2, pathEndPos, cfg);
            totalDuration += traj2.getTotalTimeSeconds();
        } catch (Exception e){
        }

        if(failed){
            DriverStation.reportWarning("Trajectory 2 generation failed!", false);
        }

        if(totalDuration == 0.0){
            return false;
        } else {
            return true;
        }

    }

    /**
     * Return the ArmPosition at a given time
     * @param time_sec
     * @return
     */
    public ArmEndEffectorState sample(double time_sec){

        if(time_sec < duration1){
            //Within the first path, all done with the starting reflex
            return ArmEndEffectorState.fromTrajState(traj1, time_sec, start.isReflex);
        } else if(time_sec < totalDuration){
            //Within the second path, all done with the ending reflex
            var curTime = time_sec - duration1;
            return ArmEndEffectorState.fromTrajState(traj2, curTime, end.get().isReflex); 
        } else {
            //Past the end of the path
            return end.get();
        }
    }

    public double getDurationSec(){
        return totalDuration;
    }

    @Override
    public List<Translation2d> getWaypoints() {
        var retList = new ArrayList<Translation2d>();
        retList.add(new Translation2d(start.x, start.y));
        retList.addAll(interiorWaypoints1);
        retList.add(new Translation2d(reflexEndPosX, reflexEndPosY));
        retList.addAll(interiorWaypoints2);
        retList.add(new Translation2d(end.get().x, end.get().y));
        return retList;
    }


}
