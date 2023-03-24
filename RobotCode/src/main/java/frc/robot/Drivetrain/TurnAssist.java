package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.Signal.Annotations.Signal;

/**
 * Generates commands to automatically turn the robot to the scoring position
 */
public class TurnAssist {

    final double ROT_SPD_RADPERSSEC = Units.degreesToRadians(220.0); //rotation rate

    Rotation2d curDesAngle = Rotation2d.fromDegrees(0.0);
    boolean autoTurnRequestedPrev = false;

    @Signal(units="radpersec")
    double curTurnCmd_radpersec = 0;

    double turnEndTime = 0.0;
    double turnStartTime = 0.0;
    @Signal(units="frac")
    double profileFrac = 0.0;
    Rotation2d turnStartAngle;

    @Signal(units="bool")
    boolean turnLeft = false;

    @Signal(units="deg")
    double desAngleDbg = 0;

    @Signal(units="deg")
    double actAngleDbg = 0;

    public double update(double manualTurnCmd_radpersec, boolean autoTurnRequested){
        Rotation2d curChassisRot = DrivetrainPoseEstimator.getInstance().getEstPose().getRotation();
        if(autoTurnRequested){
            boolean onBlue = DriverStation.getAlliance() == Alliance.Blue;
            Rotation2d targetRot = Rotation2d.fromDegrees(onBlue ? 180.0 : 0.0);

            //auto turn mode
            if(!autoTurnRequestedPrev){
                //First loop - init
                turnStartTime = Timer.getFPGATimestamp();
                turnEndTime = turnStartTime + calcProfileDur(targetRot, curChassisRot, ROT_SPD_RADPERSSEC);
                turnLeft = targetRot.minus(curChassisRot).getDegrees() > 0.0; //Should return -180 to 180
                turnStartAngle = curChassisRot;
            }

            double curTime = Timer.getFPGATimestamp() - turnStartTime;
            profileFrac = curTime / (turnEndTime - turnStartTime);

            double cmdFF = 0.0;
            if(profileFrac <= 1.0){
                cmdFF = ROT_SPD_RADPERSSEC * (turnLeft? 1.0 : -1.0);
                curDesAngle = turnStartAngle.interpolate(targetRot, profileFrac);
            } else {
                cmdFF = 0.0;
                curDesAngle = targetRot;
            }

            curTurnCmd_radpersec = cmdFF; //todo feedback

        } else {
            //Manual mode - pass through manual command
            curTurnCmd_radpersec = manualTurnCmd_radpersec;
            curDesAngle = curChassisRot;
        }


        autoTurnRequestedPrev = autoTurnRequested;

        desAngleDbg = curDesAngle.getDegrees();
        actAngleDbg = curChassisRot.getDegrees();

        return curTurnCmd_radpersec;
    }

    private double calcProfileDur(Rotation2d end, Rotation2d start, double speed){
        return Math.abs(end.minus(start).getRadians()) / speed;
    }
    
}
