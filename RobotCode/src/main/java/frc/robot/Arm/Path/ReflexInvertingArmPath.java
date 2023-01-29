package frc.robot.Arm.Path;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;
import frc.Constants;
import frc.robot.Arm.ArmEndEffectorState;
import frc.robot.Arm.ArmNamedPosition;

/**
 * Thin wrapper on wpilib's trajectory to re-orient the reference frame from
 * a field into our arm's plane. Reflex-Inverting implies the arm will fully extend
 * partway through the path, flip its reflex, then continue onward.
 */
public class ReflexInvertingArmPath implements ArmPath {


    //WPILib Trajectory
    Trajectory traj1;
    Trajectory traj2;

    ArmEndEffectorState start; 
    ArmNamedPosition end;
    double max_vel;
    double max_accel; 

    final double reflexDist = (Constants.ARM_STICK_LENGTH + Constants.ARM_BOOM_LENGTH);
    final double reflexAngleRad = Units.degreesToRadians(15.0);
    final double reflexMidPointFrac = 0.85;
    final double reflexEndPosX = reflexDist * Math.cos(reflexAngleRad);
    final double reflexEndPosY = reflexDist * Math.sin(reflexAngleRad) + Constants.ARM_BOOM_MOUNT_HIEGHT;
    final double reflexMidPosX = reflexDist * reflexMidPointFrac * Math.cos(reflexAngleRad);
    final double reflexMidPosY = reflexDist * reflexMidPointFrac * Math.sin(reflexAngleRad) + Constants.ARM_BOOM_MOUNT_HIEGHT;


    /**
     * Create a new arm trajectory from start to end with the given constraints
     * @param start
     * @param end
     * @param max_vel_mps
     * @param max_accel_mps2

     */
    public void build(ArmEndEffectorState start, ArmNamedPosition end, double max_vel_mps, double max_accel_mps2){
        this.start = start; 
        this.end = end;

        this.max_vel = max_vel_mps;
        this.max_accel = max_accel_mps2; 

        TrajectoryConfig cfg = new TrajectoryConfig(max_vel_mps, max_accel_mps2);
        cfg.addConstraint(new CentripetalAccelerationConstraint(max_accel_mps2));

        var reflexEndpoint = new ArmEndEffectorState(reflexEndPosX, reflexEndPosY);
        var reflexMidpoint = new Translation2d(reflexMidPosX, reflexMidPosY);

        ////////////////////////////////////
        // First Trajectory

        var interiorWaypoints = new ArrayList<Translation2d>();//none by default

        // Always start with slight upward motion
        interiorWaypoints.add(new Translation2d(start.x, start.y + 0.2));

        //Go to the reflex midpoint (right before actually reflexing)
        interiorWaypoints.add(reflexMidpoint);

        traj1 = TrajectoryGenerator.generateTrajectory(start.toStartPose(), interiorWaypoints, reflexEndpoint.toPoseFromOther(reflexMidpoint), cfg);

        

        interiorWaypoints = new ArrayList<Translation2d>();//none by default

        interiorWaypoints.add(reflexMidpoint);

        //If the end has a configured safe height, add in waypoints to account for it
        if(end.safeY > 0){
            var safeWaypoint1 = new Translation2d(end.pos.x, end.safeY);
            var safeWaypoint2 = new Translation2d(end.pos.x, (end.safeY + end.pos.y)/2);
            interiorWaypoints.add(safeWaypoint1);
            interiorWaypoints.add(safeWaypoint2);
        }

        traj2 = TrajectoryGenerator.generateTrajectory(reflexEndpoint.toPoseToOther(reflexMidpoint), interiorWaypoints, end.pos.toEndPose(), cfg);

        


    }

    /**
     * Return the ArmPosition at a given time
     * @param time_sec
     * @return
     */
    public ArmEndEffectorState sample(double time_sec){

        // Calculate interpolated reflex, which starts once we're near the reflex point

        if(time_sec <= traj1.getTotalTimeSeconds()){
            return ArmEndEffectorState.fromTrajState(traj1, time_sec, start.reflexFrac);
        } else {
            var curTime = time_sec - traj1.getTotalTimeSeconds();
            return ArmEndEffectorState.fromTrajState(traj2, curTime, end.pos.reflexFrac); 
        }
    }

    public double getDurationSec(){
        return traj1.getTotalTimeSeconds() + traj2.getTotalTimeSeconds();
    }

}
