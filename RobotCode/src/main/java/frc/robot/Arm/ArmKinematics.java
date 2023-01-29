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

        // Calcualte reflex fraction from knowing whether the stick is angled up or
        // down.
        double reflexFrac = isReflex(stickAngleCur) ? 1.0 : 0.0;

        // Return a complete end effector state object.
        return new ArmEndEffectorState(cur.getFirst(), cur.getSecond(), xVel, yVel, reflexFrac);

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
        var curReflex = in.reflexFrac;

        var nextX = in.x + in.xvel * 0.02;
        var nextY = in.y + in.yvel * 0.02;
        var nextReflex = in.reflexFrac; //todo need reflex rate of change

        var prevX = in.x - in.xvel * 0.02;
        var prevY = in.y - in.yvel * 0.02;
        var prevReflex = in.reflexFrac; //todo need reflex rate of change

        //Run all three through kinematics
        var cur = inverse_reflex_interpolated(curX, curY, curReflex);
        var next = inverse_reflex_interpolated(nextX, nextY, nextReflex);
        var prev = inverse_reflex_interpolated(prevX, prevY, prevReflex);

        //Calcualte angles and angular velocities
        var boomAngleDeg = Units.radiansToDegrees(cur.getFirst());
        var stickAngleDeg = Units.radiansToDegrees(cur.getSecond());
        var boomAnglularVel = Units.radiansToDegrees(next.getFirst() - prev.getFirst())/0.04;
        var stickAngularVel = Units.radiansToDegrees(next.getSecond() - prev.getSecond())/0.04;

        return new ArmAngularState(boomAngleDeg, boomAnglularVel, stickAngleDeg, stickAngularVel);   
    }

    private static Pair<Double, Double> inverse_reflex_interpolated(double x, double y, double reflexFrac){
        // Calculate the fully reflex and fully non-reflex solutions
        var retReflex = inverse_internal(x, y, true);
        var retNonReflex = inverse_internal(x, y, false);

        // Interpolate between reflex and non-reflex positions by reflex frac
        double boomAngleRad = retReflex.getFirst() * reflexFrac + retNonReflex.getFirst() * (1.0 - reflexFrac);
        double stickAngleRad = retReflex.getSecond() * reflexFrac + retNonReflex.getSecond() * (1.0 - reflexFrac);

        return Pair.of(boomAngleRad, stickAngleRad);
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
            var factor = maxRadius / reqRadius * 0.99999;
            x *= factor;
            y *= factor;
        } else if (reqRadius <= minRadius) {
            var factor = minRadius / reqRadius * 1.00001;
            x *= factor;
            y *= factor;
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

        double boomTerm2 = Math.atan(boomTerm2Num / boomTerm2Denom) * (isReflex ? 1.0 : -1.0);

        double boomAngleRad = boomTerm1 + boomTerm2;

        if(!Double.isFinite(boomAngleRad) || !Double.isFinite(stickAngleRad)){
            DriverStation.reportWarning("Cannot calculate inverse kinematics", false);
        }

        return Pair.of((Double) boomAngleRad, (Double) stickAngleRad);
    }

}
