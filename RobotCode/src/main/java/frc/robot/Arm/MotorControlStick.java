package frc.robot.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.Constants;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl.CANMotorCtrlType;
import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;

public class MotorControlStick {

    WrapperedCANMotorCtrl motorCtrl = new WrapperedCANMotorCtrl("Stick", Constants.ARM_STICK_MOTOR_CANID, CANMotorCtrlType.SPARK_MAX);

    //Feed Forward
    Calibration kF = new Calibration("Arm Stick kF", "V/degpersec", 0.1); 
    Calibration kG = new Calibration("Arm Stick kG", "V/cos(deg)", 0.0);
    Calibration kS = new Calibration("Arm Stick kS", "V", 0.1); 

    //Feedback
    Calibration kP = new Calibration("Arm Stick kP", "V/deg", 0.16);
    Calibration kI = new Calibration("Arm Stick kI", "V*sec/deg", 0.0);
    Calibration kD = new Calibration("Arm Stick kD", "V/degpersec", 0.0);

    PIDController m_pid = new PIDController(0, 0, 0);

    @Signal(units="V")
    double cmdFeedForward;
    @Signal(units="V")
    double cmdFeedBack;

    @Signal(units="deg")
    double desAngleDeg;
    @Signal(units="deg")
    double actAngleDeg;

    @Signal(units="degpersec")
    double desAngVelDegPerSec;
    @Signal(units="degpersec")
    double actAngVelDegPerSec;

    @Signal
    boolean isAngleLimited;

    public MotorControlStick(){
        motorCtrl.setBrakeMode(true);
    }

    public void setBrakeMode(boolean isBrakeMode){
        motorCtrl.setBrakeMode(isBrakeMode);
    }

    public void setCmd(ArmAngularState in){
        desAngleDeg = in.stickAngleDeg;
        desAngVelDegPerSec = in.stickAngularVel;

        //Apply command limits
        if(desAngleDeg > Constants.ARM_STICK_MAX_ANGLE_DEG){
            desAngleDeg = Constants.ARM_STICK_MAX_ANGLE_DEG;
            desAngVelDegPerSec = 0.0;
            isAngleLimited = true;
        } else if (desAngleDeg < Constants.ARM_STICK_MIN_ANGLE_DEG){
            desAngleDeg = Constants.ARM_STICK_MIN_ANGLE_DEG;
            desAngVelDegPerSec = 0.0;
            isAngleLimited = true;
        } else {
            isAngleLimited = false;
        }
    }

    public void update(ArmAngularState act_in, boolean enabled){

        actAngleDeg = act_in.stickAngleDeg;
        actAngVelDegPerSec = act_in.stickAngularVel;
        var actBoomAngleDeg = act_in.boomAngleDeg;

        //Update PID constants
        m_pid.setPID(kP.get(), kI.get(), kD.get());

        //Calculate Feed-Forward
        cmdFeedForward = Math.signum(desAngVelDegPerSec) * kS.get() + 
                         Math.cos(Units.degreesToRadians(actAngleDeg + actBoomAngleDeg)) * kG.get() + 
                         desAngVelDegPerSec * kF.get();


        cmdFeedBack = m_pid.calculate(actAngleDeg, desAngleDeg);

        if(enabled){
            motorCtrl.setVoltageCmd(-1.0 * (cmdFeedForward + cmdFeedBack)); 
        } else {
            motorCtrl.setVoltageCmd(0.0);
        }

    }
    
}
