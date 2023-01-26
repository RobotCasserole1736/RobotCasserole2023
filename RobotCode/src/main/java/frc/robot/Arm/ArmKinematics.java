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

    public static ArmState reverse(ArmEndEffectorPos in) {
        // from https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/

        var stickDenom = 2 * Constants.ARM_BOOM_LENGTH * Constants.ARM_STICK_LENGTH;
        var stickNumerator = Math.pow(in.x, 2) +
                             Math.pow(in.y, 2) -
                             Math.pow(Constants.ARM_BOOM_LENGTH , 2) -
                             Math.pow(Constants.ARM_STICK_LENGTH, 2);

        var stickAngleRad = Math.acos(stickNumerator/stickDenom) * (in.isReflex?-1.0:1.0);

        var boomTerm1 = Math.atan(in.y/in.x);

        var boomTerm2Num = Constants.ARM_STICK_LENGTH * Math.sin(stickAngleRad);
        var boomTerm2Denom = Constants.ARM_BOOM_LENGTH + Constants.ARM_STICK_LENGTH * Math.cos(stickAngleRad);

        var boomTerm2 = Math.atan(boomTerm2Num/boomTerm2Denom) * (in.isReflex?-1.0:1.0);;

        var boomAngleRad = boomTerm1 + boomTerm2;
        
        return new ArmState(boomAngleRad,stickAngleRad,null);
    }

}
