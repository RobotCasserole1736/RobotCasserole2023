package frc.robot.Arm;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import frc.Constants;

public class ArmKinematics {

    public static ArmEndEffectorState forward(ArmAngularState in) {
        //Calcualte current, previous and next angles for each joint
        var boomAngleCur = in.boomAngleDeg;
        var boomAngleNext = boomAngleCur + in.boomAnglularVel * 0.02;
        var boomAnglePrev = boomAngleCur - in.boomAnglularVel * 0.02;

        var stickAngleCur = in.stickAngleDeg;
        var stickAngleNext = stickAngleCur + in.stickAngularVel * 0.02;
        var stickAnglePrev = stickAngleCur - in.stickAngularVel * 0.02;

        //Convert current, previous, and next angles into x/y positions
        var cur = forward_internal(boomAngleCur, stickAngleCur);
        var prev = forward_internal(boomAnglePrev, stickAnglePrev);
        var next = forward_internal(boomAngleNext, stickAngleNext);

        // calcualte velocities assuming constant velocity motion from previous to next point.
        var xVel = (next.x - prev.x) / 0.04;
        var yVel = (next.y - prev.y) / 0.04;

        //Calcualte reflex fraction from knowing whether the stick is angled up or down. 
        double reflexFrac = stickAngleCur > 0.0 ? 0.0 : 1.0;

        // Return a complete end effector state object.
        return new ArmEndEffectorState(cur.x, cur.y, xVel, yVel, reflexFrac);
   

    }

    private static ArmEndEffectorState forward_internal(double boomAngleDeg, double stickAngleDeg) {

        var boomRad = Units.degreesToRadians(boomAngleDeg);
        var stickRad = Units.degreesToRadians(stickAngleDeg);

        double armX = Constants.ARM_BOOM_LENGTH * Math.cos(boomRad) + 
                      Constants.ARM_STICK_LENGTH * Math.cos(boomRad + stickRad);

        double armY = Constants.ARM_BOOM_MOUNT_HIEGHT + 
                     Constants.ARM_BOOM_LENGTH * Math.sin(boomRad) + 
                     Constants.ARM_STICK_LENGTH * Math.sin(boomRad + stickRad);

        boolean isReflex = (stickRad < 0);

        return new ArmEndEffectorState(armX, armY, isReflex);
    }

    public static ArmAngularState inverse(ArmEndEffectorState in) {

        //Calculate the fully reflex and fully non-reflex solutions
        var retReflex = inverse_internal(in.x, in.y, true);
        var retNonReflex = inverse_internal(in.x, in.y, false);

        //Interpolate between reflex and non-reflex positions by reflex frac
        var boomAngle = retReflex.getFirst() * in.reflexFrac + retNonReflex.getFirst() * (1.0 - in.reflexFrac);
        var stickAngle = retReflex.getSecond() * in.reflexFrac + retNonReflex.getSecond() * (1.0 - in.reflexFrac);

        return new ArmAngularState(Units.radiansToDegrees(boomAngle),
                            Units.radiansToDegrees(stickAngle));

    }

    private static Pair<Double, Double> inverse_internal(double x, double y, boolean isReflex){
                // from https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/

        // Reset the coordinate system back down to have the boom pivot at the origin
        // just makes the subsequent math easier.
        y = y - Constants.ARM_BOOM_MOUNT_HIEGHT;

        //Ensure the input point is "reachable" by scaling it back
        // inside the unit circle of the max extension of the arm.
        double maxRadius = Constants.ARM_BOOM_LENGTH + Constants.ARM_STICK_LENGTH;
        double minRadius = Math.abs(Constants.ARM_BOOM_LENGTH - Constants.ARM_STICK_LENGTH);
        double reqRadius = Math.sqrt(x*x + y*y);

        if(reqRadius == 0.0){
            // user was silly, give up
            x = minRadius;
            y = 0.0;
        } else if(reqRadius >= maxRadius){
            var factor = maxRadius/reqRadius;
            x *= factor;
            y *= factor;
        } else if( reqRadius <= minRadius){
            var factor = minRadius/reqRadius;
            x *= factor;
            y *= factor;
        }


        double stickDenom = 2 * Constants.ARM_BOOM_LENGTH * Constants.ARM_STICK_LENGTH;
        double stickNumerator = Math.pow(x, 2) +
                             Math.pow(y, 2) -
                             Math.pow(Constants.ARM_BOOM_LENGTH , 2) -
                             Math.pow(Constants.ARM_STICK_LENGTH, 2);

        double stickAngleRad = Math.acos(stickNumerator/stickDenom) * (isReflex?-1.0:1.0);

        double boomTerm1 = Math.atan2(y,x);

        double boomTerm2Num = Constants.ARM_STICK_LENGTH * Math.sin(stickAngleRad * (isReflex?-1.0:1.0));
        double boomTerm2Denom = Constants.ARM_BOOM_LENGTH + Constants.ARM_STICK_LENGTH * Math.cos(stickAngleRad * (isReflex?-1.0:1.0));

        double boomTerm2 = Math.atan(boomTerm2Num/boomTerm2Denom) * (isReflex?1.0:-1.0);

        double boomAngleRad = boomTerm1 + boomTerm2;
        
        return Pair.of((Double)boomAngleRad,(Double)stickAngleRad);
    }

}
