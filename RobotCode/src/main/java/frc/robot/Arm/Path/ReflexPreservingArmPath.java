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
    public boolean build(ArmEndEffectorState start, ArmNamedPosition end, double max_vel_mps, double max_accel_mps2){
        this.start = start; 
        this.end = end;

        this.max_vel = max_vel_mps;
        this.max_accel = max_accel_mps2; 

        TrajectoryConfig cfg = new TrajectoryConfig(max_vel_mps, max_accel_mps2);
        cfg.addConstraint(new CentripetalAccelerationConstraint(max_accel_mps2 * Constants.ARM_PATH_CURVATURE_FACTOR));

        interiorWaypoints = new ArrayList<Translation2d>();//none by default

        double dispX = end.get().x - start.x;
        double safeYStartX = start.x + dispX * Constants.ARM_PATH_Y_HEIGHT_X_OFFSET;
        double safeYEndX = start.x + dispX * (1.0 - Constants.ARM_PATH_Y_HEIGHT_X_OFFSET);

        Pose2d pathStartPos;
        boolean belowSafeY = end.safeY > start.y;
        // If we're outside frame perimiter and about to move horizontally, 
        // ensure we start with upward motion to clear obstacles
        if(belowSafeY){
            interiorWaypoints.add(new Translation2d(safeYStartX, end.safeY));
        }
        pathStartPos = start.toPoseToTop();


        Pose2d pathEndPos;

        if(end.safeY > 0){
            //If the end has a configured safe height, add in an additional waypoint to account for it
            interiorWaypoints.add(new Translation2d(safeYEndX, end.safeY + 0.01));
            interiorWaypoints.add(new Translation2d(end.get().x, end.safeY));
        } 

        pathEndPos = end.get().toPoseFromTop();




        var failed = true;
        try {
            traj = TrajectoryGenerator.generateTrajectory(pathStartPos, interiorWaypoints, pathEndPos, cfg);
            totalDuration = traj.getTotalTimeSeconds();
            if(traj.getStates().size() > 1){
                failed = false;
            }
        } catch (Exception e){
        } 

        if(failed){
            totalDuration = 0.0;
            DriverStation.reportWarning("Trajectory generation failed!", false);
        }

        return !failed;

    }

    /**
     * Return the ArmPosition at a given time
     * @param time_sec
     * @return
     */
    public ArmEndEffectorState sample(double time_sec){
        if(time_sec < totalDuration){
            boolean curReflex = end.get().isReflex;
            return ArmEndEffectorState.fromTrajState(traj, time_sec, curReflex);
        } else {
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
        retList.addAll(interiorWaypoints);
        retList.add(new Translation2d(end.get().x, end.get().y));
        return retList;
    }

}
