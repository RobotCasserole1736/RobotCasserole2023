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
    Calibration kV = new Calibration("Arm Boom kF", "V/degpersec", 0.13);
    Calibration kG = new Calibration("Arm Boom kG", "V/cos(deg)", 0.25);
    Calibration kS = new Calibration("Arm Boom kS", "V", 0.0);

    //Feedback
    Calibration kP = new Calibration("Arm Boom kP", "V/deg", 2.0);
    Calibration kI = new Calibration("Arm Boom kI", "V*sec/deg", 0.1);
    Calibration kD = new Calibration("Arm Boom kD", "V/degpersec", 0.0);

    Calibration brakeErrThresh = new Calibration("Arm Boom Brake Engage Allowable Error", "deg", 1.0);
    Debouncer brakeErrDbnc = new Debouncer(0.25, DebounceType.kRising);

    PIDController m_pid = new PIDController(0, 0, 0);

    @Signal(units="V")
    double cmdFeedForward;
    @Signal(units="V")
    double cmdFeedBack;

    Solenoid brakeSol = new Solenoid( PneumaticsModuleType.CTREPCM, Constants.ARM_BOOM_BRAKE_SOLENOID);

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

    public MotorControlBoom(){
        motorCtrl.setBrakeMode(true);
    }

    public void setBrakeMode(boolean isBrakeMode){
        motorCtrl.setBrakeMode(isBrakeMode);
    }

    public void setCmd(ArmAngularState testDesState){
        desAngleDeg = testDesState.boomAngleDeg;
        desAngVelDegPerSec = testDesState.boomAnglularVel;

        //Apply command limits
        if(desAngleDeg > Constants.ARM_BOOM_MAX_ANGLE_DEG){
            desAngleDeg = Constants.ARM_BOOM_MAX_ANGLE_DEG;
            desAngVelDegPerSec = 0.0;
            isAngleLimited = true;
        } else if (desAngleDeg < Constants.ARM_BOOM_MIN_ANGLE_DEG){
            desAngleDeg = Constants.ARM_BOOM_MIN_ANGLE_DEG;
            desAngVelDegPerSec = 0.0;
            isAngleLimited = true;
        } else {
            isAngleLimited = false;
        }

    }    

    public void update(ArmAngularState act_in, boolean enabled){

        actAngleDeg = act_in.boomAngleDeg;
        actAngVelDegPerSec = act_in.boomAnglularVel;

        var motorCmdV = 0.0;
        //var angleErr = Math.abs(desAngleDeg - actAngleDeg);
        //var armStationary = desAngVelDegPerSec == 0.0;
        //var engageBrake = brakeErrDbnc.calculate((angleErr < brakeErrThresh.get()) && armStationary);
        var engageBrake=false; //For now - Brake is not mounted and not needed. Never engage it, always run closed loop control.

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
                            Math.cos(Units.degreesToRadians(actAngleDeg)) * kG.get() + 
                            desAngVelDegPerSec * kV.get();

            // Update feedback command
            cmdFeedBack = m_pid.calculate(actAngleDeg, desAngleDeg);

            motorCmdV = cmdFeedForward + cmdFeedBack;

            // Brake off
            brakeSol.set(false);
        }

        // Send total command to motor;
        motorCtrl.setVoltageCmd(motorCmdV); 

        if(enabled){
            motorCtrl.setVoltageCmd(cmdFeedForward + cmdFeedBack); 
        } else {
            motorCtrl.setVoltageCmd(0.0);
        }
    }

}
