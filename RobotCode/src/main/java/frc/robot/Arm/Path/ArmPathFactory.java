package frc.robot.Arm.Path;
import frc.Constants;
import frc.robot.Arm.ArmEndEffectorState;
import frc.robot.Arm.ArmNamedPosition;

/**
 * "Factory" class responsible for making new arm paths.
 * Selects between the approprate type depending on endpoints, and 
 * passes in certain constraint constants.
 */
public class ArmPathFactory {

    public static ArmPath build(ArmEndEffectorState start, ArmNamedPosition end){
        ArmPath newPath = null;
        if(start.reflexFrac == end.get().reflexFrac){
            newPath = new ReflexPreservingArmPath();
        } else {
            newPath = new ReflexInvertingArmPath();
        }
        newPath.build(start, end, Constants.ARM_END_EFF_MAX_VEL_MPS, Constants.ARM_END_EFF_MAX_ACCEL_MPS2);
        return newPath;
    }

}
