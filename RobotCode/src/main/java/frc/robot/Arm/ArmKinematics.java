package frc.robot.Arm;

import edu.wpi.first.math.util.Units;
import frc.Constants;

public class ArmKinematics {

    public static ArmEndEffectorPos forward(ArmState in) {

        var boomRad = Units.degreesToRadians(in.boomAngleDeg);
        var stickRad = Units.degreesToRadians(in.stickAngleDeg);

        double armX = Constants.ARM_BOOM_LENGTH * Math.cos(boomRad) + 
                      Constants.ARM_STICK_LENGTH * Math.cos(boomRad + stickRad);

        double armY = Constants.ARM_BOOM_MOUNT_HIEGHT + 
                     Constants.ARM_BOOM_LENGTH * Math.sin(boomRad) + 
                     Constants.ARM_STICK_LENGTH * Math.sin(boomRad + stickRad);

        boolean isReflex = (stickRad < 0);

        return new ArmEndEffectorPos(armX, armY, isReflex);
    }

    public static ArmState inverse(ArmEndEffectorPos in) {
        // from https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/

        // Reset the coordinate system back down to have the boom pivot at the origin
        // just makes the subsequent math easier.
        double x = in.x;
        double y = in.y - Constants.ARM_BOOM_MOUNT_HIEGHT;

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


        var stickDenom = 2 * Constants.ARM_BOOM_LENGTH * Constants.ARM_STICK_LENGTH;
        var stickNumerator = Math.pow(x, 2) +
                             Math.pow(y, 2) -
                             Math.pow(Constants.ARM_BOOM_LENGTH , 2) -
                             Math.pow(Constants.ARM_STICK_LENGTH, 2);

        var stickAngleRad = Math.acos(stickNumerator/stickDenom) * (in.isReflex?-1.0:1.0);

        var boomTerm1 = Math.atan2(y,x);

        var boomTerm2Num = Constants.ARM_STICK_LENGTH * Math.sin(stickAngleRad * (in.isReflex?-1.0:1.0));
        var boomTerm2Denom = Constants.ARM_BOOM_LENGTH + Constants.ARM_STICK_LENGTH * Math.cos(stickAngleRad * (in.isReflex?-1.0:1.0));

        var boomTerm2 = Math.atan(boomTerm2Num/boomTerm2Denom) * (in.isReflex?1.0:-1.0);

        var boomAngleRad = boomTerm1 + boomTerm2;
        
        return new ArmState(Units.radiansToDegrees(boomAngleRad),
                            Units.radiansToDegrees(stickAngleRad));
    }

}
