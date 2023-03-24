package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.AllianceTransformUtils;
import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;

/**
 * Generates commands to automatically turn the robot to the scoring position
 */
public class TurnAssist {

    final double ROT_SPD_RADPERSSEC = Units.degreesToRadians(180.0); //rotation rate

    Rotation2d curDesAngle = Rotation2d.fromDegrees(0.0);
    boolean autoTurnRequestedPrev = false;

    @Signal(units="radpersec")
    double curTurnCmd_radpersec = 0;

    double turnEndTime = 0.0;
    double turnStartTime = 0.0;
    @Signal(units="frac")
    double profileFrac = 0.0;
    Rotation2d turnStartAngle;
    Rotation2d targetRotPrev;

    @Signal(units="deg")
    double desAngleDbg = 0;

    @Signal(units="deg")
    double actAngleDbg = 0;

    Calibration kP = new Calibration("TA kP", "radPerSec per Rad err", 15.0);

    public double update(double manualTurnCmd_radpersec, boolean headedDownfield, boolean autoTurnRequested){
        Rotation2d curChassisRot = DrivetrainPoseEstimator.getInstance().getEstPose().getRotation();
        Rotation2d targetRot = targetRotPrev; //default to using hte last one
        if(autoTurnRequested){
            targetRot = AllianceTransformUtils.transform(Rotation2d.fromDegrees(headedDownfield? 0.0 : 180.0)); 

            //auto turn mode
            if(!autoTurnRequestedPrev || !targetRot.equals(targetRotPrev)){
                //Re-init profile params - command has just turned on, or the target changes
                turnStartTime = Timer.getFPGATimestamp();
                turnEndTime = turnStartTime + calcProfileDur(targetRot, curChassisRot, ROT_SPD_RADPERSSEC);
                turnStartAngle = curChassisRot;
            }

            var err_rad = targetRot.minus(curChassisRot).getRadians();

            double curTime = Timer.getFPGATimestamp() - turnStartTime;
            profileFrac = Math.min(1.0, curTime / (turnEndTime - turnStartTime));

            double cmdFF = 0.0;
            if(profileFrac < 1.0){
                cmdFF = ROT_SPD_RADPERSSEC * Math.signum(err_rad);
                curDesAngle = turnStartAngle.interpolate(targetRot, profileFrac);
            } else {
                cmdFF = kP.get() * err_rad;
                curDesAngle = targetRot;
            }

            curTurnCmd_radpersec = cmdFF; 

        } else {
            //Manual mode - pass through manual command
            curTurnCmd_radpersec = manualTurnCmd_radpersec;
            curDesAngle = curChassisRot;
        }

        autoTurnRequestedPrev = autoTurnRequested;
        targetRotPrev = targetRot;

        desAngleDbg = curDesAngle.getDegrees();
        actAngleDbg = curChassisRot.getDegrees();

        return curTurnCmd_radpersec;
    }

    private double calcProfileDur(Rotation2d end, Rotation2d start, double speed){
        return Math.abs(end.minus(start).getRadians()) / speed;
    }
    
}
