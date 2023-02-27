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
        var boomAnglularVel = Units.radiansToDegrees(next.getFirst() - prev.getFirst())/0.04;
        var stickAngularVel = Units.radiansToDegrees(next.getSecond() - prev.getSecond())/0.04;

        return new ArmAngularState(boomAngleDeg, boomAnglularVel, stickAngleDeg, stickAngularVel);   
    }

    private static Pair<Double, Double> inverse_internal(double x, double y, boolean isReflex) {
        // from
        // https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/

        // Reset the coordinate system back down to have the boom pivot at the origin
        // just makes the subsequent math easier.
        y = y - Constants.ARM_BOOM_MOUNT_HIEGHT;

        // Ensure the input point is "reachable" by scaling it back
        // inside the unit circle of the max extension of the arm.
        double maxRadius = Constants.ARM_BOOM_LENGTH + Constants.ARM_STICK_LENGTH;
        double minRadius = Math.abs(Constants.ARM_BOOM_LENGTH - Constants.ARM_STICK_LENGTH);
        double reqRadius = Math.sqrt(x * x + y * y);

        if (reqRadius == 0.0) {
            // user was silly, give up
            x = minRadius;
            y = 0.0;
        } else if (reqRadius >= maxRadius) {
            //Requested point beyond the reach of the arm
            // Project the point back into the reachable area
            var angle = Math.atan2(y, x);
            x = maxRadius * Math.cos(angle) * 0.99999999; //Fudge factor because floating point numbers might still round to the wrong side of the circle
            y = maxRadius * Math.sin(angle) * 0.99999999;
        } else if (reqRadius <= minRadius) {
            //Requested point inside the "unreachable" circle near
            // the pivot point. 
            // Project the point back out into the reachable area
            var angle = Math.atan2(y, x);
            x = minRadius * Math.cos(angle) * 1.0000000000001; //Fudge Factor because floating point numbers might still round to the wrong side of the circle
            y = minRadius * Math.sin(angle) * 1.0000000000001;
        }

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
            DriverStation.reportWarning("Cannot calculate inverse kinematics", false);
        }

        return Pair.of((Double) boomAngleRad, (Double) stickAngleRad);
    }

}
