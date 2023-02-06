package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

/**
 * Encapsulates the main components of what the pathplanner libraries
 * communicate into the drivetrain control class
 */
public class SwerveTrajectoryCmd {
    
    // Drivetrain chassis desired motion (translation & curvature of the center of mass)
    public Trajectory.State desTrajState; 
    // Desired holonomic angle
    public Rotation2d desAngle;
    // Desired holonomic angular velocity (how fast the chassis is spinning) 
    public Rotation2d desAngVel;

    public SwerveTrajectoryCmd(Trajectory.State desTrajState, Rotation2d desAngle, Rotation2d desAngVel){
        this.desTrajState = desTrajState;
        this.desAngle = desAngle;
        this.desAngVel = desAngVel;
    }
    
}
