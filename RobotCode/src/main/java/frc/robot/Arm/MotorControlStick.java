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
    Calibration kV = new Calibration("Arm Stick kF", "V/radpersec", 0.0);
    Calibration kG = new Calibration("Arm Stick kG", "V/rad", 0.0);
    Calibration kS = new Calibration("Arm Stick kS", "V", 0.0);

    //Feedback
    Calibration kP = new Calibration("Arm Stick kP", "V/rad", 12.0/5.0);
    Calibration kI = new Calibration("Arm Stick kI", "V*sec/rad", 0.0);
    Calibration kD = new Calibration("Arm Stick kD", "V/radpersec", 0.0);

    PIDController m_pid = new PIDController(0, 0, 0);

    @Signal(units="V")
    double cmdFeedForward;
    @Signal(units="V")
    double cmdFeedBack;

    @Signal
    double desAngleDeg;
    @Signal
    double actAngleDeg;

    @Signal
    double desAngVelDegPerSec;
    @Signal
    double actAngVelDegPerSec;

    public void setCmd(ArmAngularState in){
        //todo - save off the right angles and velocities for this motor
        desAngleDeg = in.stickAngleDeg;
        desAngVelDegPerSec = in.stickAngularVel;
    }

    public void update(ArmAngularState act_in){

        actAngleDeg = act_in.stickAngleDeg;
        actAngVelDegPerSec = act_in.stickAngularVel;

        //Update PID constants
        m_pid.setPID(kP.get(), kI.get(), kD.get());

        //Calculate Feed-Forward
        cmdFeedForward = Math.signum(desAngVelDegPerSec) * kS.get() + 
                         Math.cos(Units.degreesToRadians(desAngleDeg)) * kG.get() + 
                         desAngVelDegPerSec * kV.get();


        cmdFeedBack = m_pid.calculate(actAngleDeg, desAngleDeg);

        motorCtrl.setVoltageCmd(cmdFeedForward + cmdFeedBack); 

    }
    
}
