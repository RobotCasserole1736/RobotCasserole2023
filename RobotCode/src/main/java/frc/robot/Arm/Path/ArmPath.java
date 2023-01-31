package frc.robot.Arm.Path;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Arm.ArmEndEffectorState;
import frc.robot.Arm.ArmNamedPosition;

/**
 * Thin wrapper on wpilib's trajectory to re-orient the reference frame from
 * a field into our arm's plane
 */
public interface ArmPath {

    public static final double MIN_PATHPLAN_DIST_M = 0.02;

    /**
     * Create a new arm trajectory from start to end with the given constraints
     * @param start
     * @param end
     * @param max_vel_mps
     * @param max_accel_mps2

     */
    public abstract void build(ArmEndEffectorState start, ArmNamedPosition end, double max_vel_mps, double max_accel_mps2);

    /**
     * Return the ArmPosition at a given time
     * @param time_sec
     * @return
     */
    public abstract ArmEndEffectorState sample(double time_sec);

    /**
     * @return the duration of the path in seconds
     */
    public abstract double getDurationSec();

    /**
     * @return the duration of the path in seconds
     */
    public abstract List<Translation2d> getWaypoints();

}
