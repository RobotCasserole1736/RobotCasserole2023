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
    Calibration kV = new Calibration("Arm Stick kF", "V/degpersec", 0.13);
    Calibration kG = new Calibration("Arm Stick kG", "V/cos(deg)", 0.1);
    Calibration kS = new Calibration("Arm Stick kS", "V", 0.0);

    //Feedback
    Calibration kP = new Calibration("Arm Stick kP", "V/deg", 2.0);
    Calibration kI = new Calibration("Arm Stick kI", "V*sec/deg", 0.1);
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

    public void setCmd(ArmAngularState in){
        //todo - save off the right angles and velocities for this motor
        desAngleDeg = in.stickAngleDeg;
        desAngVelDegPerSec = in.stickAngularVel;
    }

    public void update(ArmAngularState act_in){

        actAngleDeg = act_in.stickAngleDeg;
        actAngVelDegPerSec = act_in.stickAngularVel;
        var actBoomAngleDeg = act_in.boomAngleDeg;

        //Update PID constants
        m_pid.setPID(kP.get(), kI.get(), kD.get());

        //Calculate Feed-Forward
        cmdFeedForward = Math.signum(desAngVelDegPerSec) * kS.get() + 
                         -1.0 * Math.cos(Units.degreesToRadians(desAngleDeg + actBoomAngleDeg)) * kG.get() + 
                         desAngVelDegPerSec * kV.get();


        cmdFeedBack = m_pid.calculate(actAngleDeg, desAngleDeg);

        motorCtrl.setVoltageCmd(cmdFeedForward + cmdFeedBack); 

    }
    
}
