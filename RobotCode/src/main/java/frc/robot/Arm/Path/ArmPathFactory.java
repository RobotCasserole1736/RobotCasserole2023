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
        if(start.isReflex == end.get().isReflex){
            newPath = new ReflexPreservingArmPath();
        } else {
            newPath = new ReflexInvertingArmPath();
        }
        var success = newPath.build(start, end, Constants.ARM_END_EFF_MAX_VEL_MPS, Constants.ARM_END_EFF_MAX_ACCEL_MPS2);

        if(!success){
            //generation failed. Fallback on a simpler strategy
            newPath = new LinearInterpolatedArmPath();
            newPath.build(start, end, Constants.ARM_END_EFF_MAX_VEL_MPS * 0.3, Constants.ARM_END_EFF_MAX_ACCEL_MPS2);
        }
        return newPath;
    }

}
