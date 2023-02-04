package frc.robot.Arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class ArmEndEffectorState {
    public double x;
    public double y;
    public double xvel;
    public double yvel;
    public double reflexFrac; //1.0 is fully reflex, 0.0 is fully non-reflex

    //TODO - add something here to fiture out if we should attempt to achieve the solution through a "concave" or "convex" joint solution

    public ArmEndEffectorState(double x, double y, boolean isReflex){
        this.x = x;
        this.y = y;
        this.xvel = 0;
        this.yvel = 0;
        this.reflexFrac = isReflex ? 1.0 : 0.0;
    }

    public ArmEndEffectorState(double x, double y, double reflexFrac){
        this.x = x;
        this.y = y;
        this.xvel = 0;
        this.yvel = 0;
        this.reflexFrac = reflexFrac;
    }

    public ArmEndEffectorState(double x, double y, double xvel, double yvel, double reflexFrac){
        this.x = x;
        this.y = y;
        this.xvel = xvel;
        this.yvel = yvel;
        this.reflexFrac = reflexFrac;
    }

    public ArmEndEffectorState(double x, double y){
        this.x = x;
        this.y = y;
        this.xvel = 0;
        this.yvel = 0;
        this.reflexFrac = 0.0;
    }

    ArmEndEffectorState(){
        this.x = 0;
        this.y = 0;
        this.reflexFrac = 0;
    }

    public double distTo(ArmEndEffectorState other){
        double x2 = Math.pow(this.x - other.x, 2);
        double y2 = Math.pow(this.y - other.y, 2);
        return Math.sqrt(x2 + y2);
    }

    public ArmEndEffectorState interpolateTo(ArmEndEffectorState other, double frac){
        double fracInv = (1.0 - frac);
        double x = other.x * frac + this.x * fracInv;
        double y = other.y * frac + this.y * fracInv;
        double reflexFrac = other.reflexFrac * frac + this.reflexFrac * fracInv;
        return new ArmEndEffectorState(x, y, reflexFrac);
    }

    public ArmEndEffectorState interpolateTo(ArmEndEffectorState other, double frac, double linearVel){
        double fracInv = (1.0 - frac);
        double x = other.x * frac + this.x * fracInv;
        double y = other.y * frac + this.y * fracInv;
        double reflexFrac = other.reflexFrac * frac + this.reflexFrac * fracInv;
        var velDir = new Rotation2d(other.x - this.x, other.y - this.y);
        double xVel = linearVel * velDir.getCos();
        double yVel = linearVel * velDir.getSin();
        return new ArmEndEffectorState(x, y, xVel, yVel, reflexFrac);
    }

    //TODO - refine these 
    // Requirements: rather than two methods for start/end, we should 
    // have a single method that supports confguring whether you want to approach the position
    // from the top or bottom or maybe side?


    public Pose2d toPoseToTop(){
        return new Pose2d(this.x, this.y, Rotation2d.fromDegrees(90.0));
    }

    public Pose2d toPoseFromTop(){
        return new Pose2d(this.x, this.y, Rotation2d.fromDegrees(270.0));
    }

    public Pose2d toPoseFromOther(Translation2d other){
        var rot = new Rotation2d(this.x - other.getX(), this.y - other.getY());
        return new Pose2d(this.x, this.y, rot);
    }

    public Pose2d toPoseFromOther(ArmEndEffectorState other){
        var rot = new Rotation2d(this.x - other.x, this.y - other.y);
        return new Pose2d(this.x, this.y, rot);
    }

    public Pose2d toPoseToOther(Translation2d other){
        var rot = new Rotation2d(other.getX() - this.x, other.getY() - this.y);
        return new Pose2d(this.x, this.y, rot);
    }

    public Pose2d toPoseToOther(ArmNamedPosition other){
        var rot = new Rotation2d(other.get().x - this.x, other.get().y - this.y);
        return new Pose2d(this.x, this.y, rot);
    }

    public static ArmEndEffectorState fromTrajState(Trajectory traj, double time, double reflexFrac){
        var pos = traj.sample(time).poseMeters;
        var pos_prev = traj.sample(time - 0.02).poseMeters;
        var pos_next = traj.sample(time + 0.02).poseMeters;
        double velx = (pos_next.getX() - pos_prev.getX()) / 0.04;
        double vely = (pos_next.getY() - pos_prev.getY()) / 0.04;
        return new ArmEndEffectorState(pos.getX(), pos.getY(), velx, vely, reflexFrac);
    }


    @Override
    public boolean equals(Object o){
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        ArmEndEffectorState other = (ArmEndEffectorState)o;
        return this.x == other.x && this.y == other.y && this.reflexFrac == other.reflexFrac;
    }
}
