package frc.robot.Arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class ArmEndEffectorPos {
    public double x;
    public double y;
    public double reflexFrac; //1.0 is fully reflex, 0.0 is fully non-reflex

    //TODO - we just added isReflex and haven't used it at all yet

    // TODO - how do we handle reflex transitions? Do we need double reflexFrac that sweeps smoothly from 0 to 1

    // TODO - allow configuraiton of a "safe height" - minimum Y value the end affector must travel to before traveling in the X direction

    //TODO - add something here to fiture out if we should attempt to achieve the solution through a "concave" or "convex" joint solution

    public ArmEndEffectorPos(double x, double y, boolean isReflex){
        this.x = x;
        this.y = y;
        this.reflexFrac = isReflex ? 1.0 : 0.0;
    }

    public ArmEndEffectorPos(double x, double y, double reflexFrac){
        this.x = x;
        this.y = y;
        this.reflexFrac = reflexFrac;
    }

    public ArmEndEffectorPos(double x, double y){
        this.x = x;
        this.y = y;
        this.reflexFrac = 0.0;
    }

    ArmEndEffectorPos(){
        this.x = 0;
        this.y = 0;
        this.reflexFrac = 0;
    }

    double distTo(ArmEndEffectorPos other){
        double x2 = Math.pow(this.x - other.x, 2);
        double y2 = Math.pow(this.y - other.y, 2);
        return Math.sqrt(x2 + y2);
    }

    ArmEndEffectorPos interpolateTo(ArmEndEffectorPos other, double frac){
        double fracInv = (1.0 - frac);
        double x = other.x * frac + this.x * fracInv;
        double y = other.y * frac + this.y * fracInv;
        double reflexFrac = other.reflexFrac * frac + this.reflexFrac * fracInv;
        return new ArmEndEffectorPos(x, y, reflexFrac);
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

    static ArmEndEffectorPos fromTrajState(State in, double reflexFrac){
        var tmp = in.poseMeters;
        return new ArmEndEffectorPos(tmp.getX(), tmp.getY(), reflexFrac);
    }


    @Override
    public boolean equals(Object o){
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        ArmEndEffectorPos other = (ArmEndEffectorPos)o;
        return this.x == other.x && this.y == other.y && this.reflexFrac == other.reflexFrac;
    }
}
