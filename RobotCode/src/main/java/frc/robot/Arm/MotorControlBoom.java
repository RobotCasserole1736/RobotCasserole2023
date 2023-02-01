package frc.robot.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.Constants;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl.CANMotorCtrlType;
import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;

public class MotorControlBoom {


    WrapperedCANMotorCtrl motorCtrl = new WrapperedCANMotorCtrl("Boom", Constants.ARM_BOOM_MOTOR_CANID, CANMotorCtrlType.SPARK_MAX);

    //Feed Forward
    Calibration kV = new Calibration("Arm Boom kF", "V/radpersec", 0.0);
    Calibration kG = new Calibration("Arm Boom kG", "V/rad", 0.0);
    Calibration kS = new Calibration("Arm Boom kS", "V", 0.0);

    //Feedback
    Calibration kP = new Calibration("Arm Boom kP", "V/rad", 12.0/5.0);
    Calibration kI = new Calibration("Arm Boom kI", "V*sec/rad", 0.0);
    Calibration kD = new Calibration("Arm Boom kD", "V/radpersec", 0.0);

    Calibration brakeErrThresh = new Calibration("Arm Boom Brake Engage Allowable Error", "deg", 1.0);
    Debouncer brakeErrDbnc = new Debouncer(0.25, DebounceType.kRising);

    PIDController m_pid = new PIDController(0, 0, 0);

    @Signal(units="V")
    double cmdFeedForward;
    @Signal(units="V")
    double cmdFeedBack;

    Solenoid brakeSol = new Solenoid( PneumaticsModuleType.CTREPCM, Constants.ARM_BOOM_BRAKE_SOLENOID);

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
        desAngleDeg = in.boomAngleDeg;
        desAngVelDegPerSec = in.boomAnglularVel;
    }    

    public void update(ArmAngularState act_in){

        actAngleDeg = act_in.boomAngleDeg;
        actAngVelDegPerSec = act_in.boomAnglularVel;

        var motorCmdV = 0.0;
        var angleErr = Math.abs(desAngleDeg - actAngleDeg);
        var engageBrake = brakeErrDbnc.calculate(angleErr < brakeErrThresh.get());

        if(engageBrake){
            //Brake engaged. 
            // Don't run the motor 
            motorCmdV = 0.0;
            // Brake on
            brakeSol.set(true);

        } else {
            // Brake disengaged. Normal arm control from motor.
            // update PID Controller Constants
            m_pid.setPID(kP.get(), kI.get(), kD.get());

            // Calculate Feed-Forward
            cmdFeedForward = Math.signum(desAngVelDegPerSec) * kS.get() + 
                            Math.cos(Units.degreesToRadians(desAngleDeg)) * kG.get() + 
                            desAngVelDegPerSec * kV.get();

            // Update feedback command
            cmdFeedBack = m_pid.calculate(actAngleDeg, desAngleDeg);

            motorCmdV = cmdFeedForward + cmdFeedBack;

            // Brake off
            brakeSol.set(true);
        }


        // Send total command to motor;
        motorCtrl.setVoltageCmd(motorCmdV); 
    }
}
