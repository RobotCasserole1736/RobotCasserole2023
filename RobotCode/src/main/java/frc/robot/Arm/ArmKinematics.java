package frc.robot.Arm;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.Constants;

public class ArmKinematics {

    public static ArmEndEffectorState forward(ArmAngularState in) {
        // Calcualte current, previous and next angles for each joint
        var boomAngleCur = in.boomAngleDeg;
        var boomAngleNext = boomAngleCur + in.boomAnglularVel * 0.02;
        var boomAnglePrev = boomAngleCur - in.boomAnglularVel * 0.02;

        var stickAngleCur = in.stickAngleDeg;
        var stickAngleNext = stickAngleCur + in.stickAngularVel * 0.02;
        var stickAnglePrev = stickAngleCur - in.stickAngularVel * 0.02;

        // Convert current, previous, and next angles into x/y positions
        var cur = forward_internal(boomAngleCur, stickAngleCur);
        var prev = forward_internal(boomAnglePrev, stickAnglePrev);
        var next = forward_internal(boomAngleNext, stickAngleNext);

        // calcualte velocities assuming constant velocity motion from previous to next
        // point.
        var xVel = (next.getFirst() - prev.getFirst()) / 0.04;
        var yVel = (next.getSecond() - prev.getSecond()) / 0.04;

        // Return a complete end effector state object.
        return new ArmEndEffectorState(cur.getFirst(), cur.getSecond(), xVel, yVel, isReflex(stickAngleCur));

    }

    private static boolean isReflex(double stickAngleDeg){
        return (stickAngleDeg < 0);
    }

    private static Pair<Double, Double> forward_internal(double boomAngleDeg, double stickAngleDeg) {

        var boomRad = Units.degreesToRadians(boomAngleDeg);
        var stickRad = Units.degreesToRadians(stickAngleDeg);

        double armX = Constants.ARM_BOOM_LENGTH * Math.cos(boomRad) +
                Constants.ARM_STICK_LENGTH * Math.cos(boomRad + stickRad);

        double armY = Constants.ARM_BOOM_MOUNT_HIEGHT +
                Constants.ARM_BOOM_LENGTH * Math.sin(boomRad) +
                Constants.ARM_STICK_LENGTH * Math.sin(boomRad + stickRad);

        return Pair.of(armX, armY);
    }

    public static ArmAngularState inverse(ArmEndEffectorState in) {

        //Calculate current, previous, and next x/y positions
        var curX = in.x;
        var curY = in.y;
        var curReflex = in.isReflex;

        var nextX = in.x + in.xvel * 0.02;
        var nextY = in.y + in.yvel * 0.02;

        var prevX = in.x - in.xvel * 0.02;
        var prevY = in.y - in.yvel * 0.02;

        //Run all three through kinematics
        var cur = inverse_internal(curX, curY, curReflex);
        var next = inverse_internal(nextX, nextY, curReflex);
        var prev = inverse_internal(prevX, prevY, curReflex);

        //Calcualte angles and angular velocities
        var boomAngleDeg = Units.radiansToDegrees(cur.getFirst());
        var stickAngleDeg = Units.radiansToDegrees(cur.getSecond());
        var boomAnglularVel = 0.0;
        var stickAngularVel = 0.0;
        var boomAngularAccel = 0.0;
        var stickAngularAccel = 0.0;
        if(next != null && prev != null){
            // Average Vel
            boomAnglularVel = toAngVelDegPerSec(next.getFirst(), prev.getFirst(), 0.04);
            stickAngularVel = toAngVelDegPerSec(next.getSecond(), prev.getSecond(), 0.04);

            // Average Accel
            boomAngularAccel = toAngAccelDegPerSec2(next.getFirst(), cur.getFirst(), prev.getFirst(), 0.02);
            stickAngularAccel = toAngAccelDegPerSec2(next.getSecond(), cur.getSecond(), prev.getSecond(), 0.02);
        }
        
        return new ArmAngularState(boomAngleDeg, boomAnglularVel, stickAngleDeg, stickAngularVel, boomAngularAccel, stickAngularAccel);   
    }

    private static double toAngVelDegPerSec(double next, double first, double delta_time){
        return Units.radiansToDegrees(next - first)/delta_time;
    }

    private static double toAngAccelDegPerSec2(double next, double cur, double prev, double delta_time){
        var vel_f =  Units.radiansToDegrees(next - cur)/delta_time;
        var vel_i =  Units.radiansToDegrees(cur - prev)/delta_time;
        return (vel_f - vel_i) / delta_time;
    }


    private static Pair<Double, Double> inverse_internal(double x, double y, boolean isReflex) {
        // from
        // https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/

        // Reset the coordinate system back down to have the boom pivot at the origin
        // just makes the subsequent math easier.
        y = y - Constants.ARM_BOOM_MOUNT_HIEGHT;

        double stickDenom = 2 * Constants.ARM_BOOM_LENGTH * Constants.ARM_STICK_LENGTH;
        double stickNumerator = Math.pow(x, 2) +
                Math.pow(y, 2) -
                Math.pow(Constants.ARM_BOOM_LENGTH, 2) -
                Math.pow(Constants.ARM_STICK_LENGTH, 2);

        double stickAngleRad = Math.acos(stickNumerator / stickDenom) * (isReflex ? -1.0 : 1.0);

        double boomTerm1 = Math.atan2(y, x);

        double boomTerm2Num = Constants.ARM_STICK_LENGTH * Math.sin(stickAngleRad * (isReflex ? -1.0 : 1.0));
        double boomTerm2Denom = Constants.ARM_BOOM_LENGTH
                + Constants.ARM_STICK_LENGTH * Math.cos(stickAngleRad * (isReflex ? -1.0 : 1.0));

        double boomTerm2 = Math.atan2(boomTerm2Num , boomTerm2Denom) * (isReflex ? 1.0 : -1.0);

        double boomAngleRad = boomTerm1 + boomTerm2;

        if(!Double.isFinite(boomAngleRad) || !Double.isFinite(stickAngleRad)){
            return null; //Unreachable position
        } else {
            return Pair.of((Double) boomAngleRad, (Double) stickAngleRad);
        }

    }

}
