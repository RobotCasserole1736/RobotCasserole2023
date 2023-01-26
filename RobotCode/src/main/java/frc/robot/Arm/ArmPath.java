package frc.robot.Arm;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/**
 * Thin wrapper on wpilib's trajectory to re-orient the reference frame from
 * a field into our arm's plane
 */
public class ArmPath {


    //Time to linear position
    Trajectory traj;

    ArmEndEffectorPos start; 
    ArmEndEffectorPos end;
    double max_vel;
    double max_accel; 

    /**
     * Create a new arm trajectory from start to end with the given constraints
     * @param start
     * @param end
     * @param max_vel_mps
     * @param max_accel_mps2

     */
    public ArmPath(ArmEndEffectorPos start, ArmEndEffectorPos end, double max_vel_mps, double max_accel_mps2){
        this.start = start; 
        this.end = end;

        this.max_vel = max_vel_mps;
        this.max_accel = max_accel_mps2; 

        var interiorWaypoints = new ArrayList<Translation2d>();//none?
        TrajectoryConfig cfg = new TrajectoryConfig(max_vel_mps, max_accel_mps2);
        traj = TrajectoryGenerator.generateTrajectory(start.toStartPose(), interiorWaypoints, end.toEndPose(), cfg);

    }

    /**
     * Return the ArmPosition at a given time
     * @param time_sec
     * @return
     */
    public ArmEndEffectorPos sample(double time_sec){
        //TODO - Trajectory State includes velocity and direction information, maybe that needs to be exposed too?
        // Right now this will strip all that away and jsut return x/y position
        return ArmEndEffectorPos.fromTrajState(traj.sample(time_sec), end.isReflex);
    }

    public double getDurationSec(){
        return traj.getTotalTimeSeconds();
    }

}
