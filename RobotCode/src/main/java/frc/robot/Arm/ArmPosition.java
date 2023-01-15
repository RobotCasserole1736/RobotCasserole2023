package frc.robot.Arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class ArmPosition {
    double x;
    double y;

    // TODO - allow configuraiton of a "safe height" - minimum Y value the end affector must travel to before traveling in the X direction

    //TODO - add something here to fiture out if we should attempt to achieve the solution through a "concave" or "convex" joint solution

    ArmPosition(double x, double y){
        this.x = x;
        this.y = y;
    }

    double distTo(ArmPosition other){
        double x2 = Math.pow(this.x - other.x, 2);
        double y2 = Math.pow(this.y - other.y, 2);
        return Math.sqrt(x2 + y2);
    }

    ArmPosition interpolateTo(ArmPosition other, double frac){
        double fracInv = (1.0 - frac);
        double x = other.x * frac + this.x * fracInv;
        double y = other.y * frac + this.y * fracInv;
        return new ArmPosition(x, y);
    }

    //TODO - refine these 
    // Requirements: rather than two methods for start/end, we should 
    // have a single method that supports confguring whether you want to approach the position
    // from the top or bottom or maybe side?


    Pose2d toStartPose(){
        return new Pose2d(this.x, this.y, Rotation2d.fromDegrees(90.0));
    }

    Pose2d toEndPose(){
        return new Pose2d(this.x, this.y, Rotation2d.fromDegrees(270.0));
    }

    static ArmPosition fromTrajState(State in){
        var tmp = in.poseMeters;
        return new ArmPosition(tmp.getX(), tmp.getY());
    }
}
