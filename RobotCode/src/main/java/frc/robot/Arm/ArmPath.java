package frc.robot.Arm;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import frc.Constants;

/**
 * Thin wrapper on wpilib's trajectory to re-orient the reference frame from
 * a field into our arm's plane
 */
public class ArmPath {


    //WPILib Trajectory
    Trajectory traj;

    ArmEndEffectorState start; 
    ArmNamedPosition end;
    double max_vel;
    double max_accel; 

    final double reflexTransitionTimeSec = 1.0;
    final double reflexHeightM = Constants.ARM_BOOM_MOUNT_HIEGHT;
    final double reflexStartPosX = (Constants.ARM_STICK_LENGTH + Constants.ARM_BOOM_LENGTH);
    double reflexStartTime = 0;
    boolean reflexStarted = false;
    boolean reflexFinished = false;

    TrapezoidProfile reflexTrap;

    /**
     * Create a new arm trajectory from start to end with the given constraints
     * @param start
     * @param end
     * @param max_vel_mps
     * @param max_accel_mps2

     */
    public ArmPath(ArmEndEffectorState start, ArmNamedPosition end, double max_vel_mps, double max_accel_mps2){
        this.start = start; 
        this.end = end;

        this.max_vel = max_vel_mps;
        this.max_accel = max_accel_mps2; 

        var interiorWaypoints = new ArrayList<Translation2d>();//none by default

        //If we're switching reflex, add an extra "full extension" waypoint.
        if(start.reflexFrac == end.pos.reflexFrac){
            interiorWaypoints.add(new Translation2d(Constants.ARM_BOOM_LENGTH + Constants.ARM_STICK_LENGTH, reflexHeightM));
            reflexTrap = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.0/reflexTransitionTimeSec, 1.0/reflexTransitionTimeSec), new TrapezoidProfile.State(1.0, 0.0));
        } 
        
        //If the end has a configured safe height, add in an additional waypoint to account for it
        if(end.safeY > 0){
            var safeWaypoint = new Translation2d(end.pos.x, end.safeY);
            interiorWaypoints.add(safeWaypoint);
        }

        TrajectoryConfig cfg = new TrajectoryConfig(max_vel_mps, max_accel_mps2);
        cfg.addConstraint(new CentripetalAccelerationConstraint(max_accel_mps2));
        
        traj = TrajectoryGenerator.generateTrajectory(start.toStartPose(), interiorWaypoints, end.pos.toEndPose(), cfg);

    }

    /**
     * Return the ArmPosition at a given time
     * @param time_sec
     * @return
     */
    public ArmEndEffectorState sample(double time_sec){

        // Calculate interpolated reflex, which starts once we're near the reflex point

        if(reflexStarted == false){
            var curPosCmd = traj.sample(time_sec);
            if(curPosCmd.poseMeters.getX() > reflexStartPosX){
                reflexStartTime = time_sec;
                reflexStarted = true;
            }
        } else {
            if(time_sec >= reflexStartTime + reflexTransitionTimeSec){
                reflexFinished = true;
            }
        }

        double curReflex = 0.0;
        if(reflexStarted == false){
            curReflex = start.reflexFrac;
        } else if(reflexFinished == true){
            curReflex = end.pos.reflexFrac;
        } else {
            //in transition
            var timeFrac = reflexTrap.calculate(time_sec - reflexStartTime).position;
            curReflex = start.reflexFrac * (1.0 - timeFrac) + end.pos.reflexFrac * timeFrac;
        }


        return ArmEndEffectorState.fromTrajState(traj, time_sec, curReflex);
    }

    public double getDurationSec(){
        return traj.getTotalTimeSeconds();
    }

}
