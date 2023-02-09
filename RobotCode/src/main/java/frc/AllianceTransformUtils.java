package frc;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Utilities to help transform from blue alliance to red if needed)
 * We went rogue and chose a coordinate system where the origin is always in the 
 * bottom left on the blue alliance
 */
public class AllianceTransformUtils{

    public static double transformX(double in){
        if(DriverStation.getAlliance() == Alliance.Red){
            return Constants.FIELD_LENGTH_M - in;
        } else {
            return in;
        }
    }

    public static double transformY(double in){
        return in;
    }

    public static Rotation2d transform(Rotation2d in){
        if(DriverStation.getAlliance() == Alliance.Red){
            return Rotation2d.fromDegrees(180).minus(in);
        } else {
            return in;
        }
    }

    public static Translation2d transform(Translation2d in){
        if(DriverStation.getAlliance() == Alliance.Red){
            return new Translation2d(AllianceTransformUtils.transformX(in.getX()), in.getY());
        } else {
            return in;
        }
    }

    public static Transform2d transform(Transform2d in){
        if(DriverStation.getAlliance() == Alliance.Red){
            var trans = AllianceTransformUtils.transform(in.getTranslation());
            var rot = AllianceTransformUtils.transform(in.getRotation());
            return new Transform2d(trans, rot);
        } else {
            return in;
        }
    }

    public static Pose2d transform(Pose2d in){
        if(DriverStation.getAlliance() == Alliance.Red){
            var trans = AllianceTransformUtils.transform(in.getTranslation());
            var rot = AllianceTransformUtils.transform(in.getRotation());
            return new Pose2d(trans, rot);
        } else {
            return in;
        }
    }

    public static PathPlannerState transform(PathPlannerState in){
        if(DriverStation.getAlliance() == Alliance.Red){
            var retval = new PathPlannerState();
            retval.holonomicRotation = AllianceTransformUtils.transform(in.holonomicRotation);
            retval.angularVelocityRadPerSec = -1.0 * in.angularVelocityRadPerSec;
            retval.curvatureRadPerMeter = -1.0 * in.curvatureRadPerMeter;
            retval.holonomicAngularVelocityRadPerSec = -1.0 * in.holonomicAngularVelocityRadPerSec; 
            retval.poseMeters = AllianceTransformUtils.transform(in.poseMeters);
            retval.timeSeconds = in.timeSeconds;
            retval.velocityMetersPerSecond = in.velocityMetersPerSecond;
            retval.accelerationMetersPerSecondSq = retval.accelerationMetersPerSecondSq;
            return retval;
        } else {
            return in;
        }
    }

    

}