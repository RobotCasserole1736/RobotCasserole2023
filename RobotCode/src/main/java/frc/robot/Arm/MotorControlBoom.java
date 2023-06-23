package frc.robot.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.Constants;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl.CANMotorCtrlType;
import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;
import frc.lib.Util.MapLookup2D;

public class MotorControlBoom {


    WrapperedCANMotorCtrl motorCtrl = new WrapperedCANMotorCtrl("Boom", Constants.ARM_BOOM_MOTOR_CANID, CANMotorCtrlType.SPARK_MAX);

    //Feed Forward
    Calibration kF = new Calibration("Arm Boom kF", "V/degpersec", 0.05);
    Calibration kG = new Calibration("Arm Boom kG", "V/cos(deg)", 0.3);
    Calibration kS = new Calibration("Arm Boom kS", "V", 0.05);

    //Feedback
    Calibration kP = new Calibration("Arm Boom kP", "V/deg", 0.2);
    Calibration kI = new Calibration("Arm Boom kI", "V*sec/deg", 0.0);
    Calibration kD = new Calibration("Arm Boom kD", "V/degpersec", 0.00);

    // Gain schedule P 
    final double pDeazoneErrDeg = 1.375/2.0; //keep a very very tiny deadzone
    final double pDeadzonTransitionWidth = pDeazoneErrDeg/4.0;
    MapLookup2D closedLoopGainSchedule;

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
    
    @Signal(units="degpersec2")
    double desAngAccelDegPerSec;

    @Signal
    boolean isAngleLimited;


    public MotorControlBoom(){
        motorCtrl.setBrakeMode(true);

        //Gain schedule P to have zero value in the deadzone
        // but get more powerful as error gets larger
        closedLoopGainSchedule = new MapLookup2D();
        closedLoopGainSchedule.insertNewPoint(-180, 1.0);
        closedLoopGainSchedule.insertNewPoint(-pDeazoneErrDeg - pDeadzonTransitionWidth, 1.0);
        closedLoopGainSchedule.insertNewPoint(-pDeazoneErrDeg, 0.0);
        closedLoopGainSchedule.insertNewPoint(pDeazoneErrDeg, 0.0);
        closedLoopGainSchedule.insertNewPoint(pDeazoneErrDeg + pDeadzonTransitionWidth, 1.0);
        closedLoopGainSchedule.insertNewPoint(180, 1.0);
    }

    public void setBrakeMode(boolean isBrakeMode){
        motorCtrl.setBrakeMode(isBrakeMode);
    }

    public void setCmd(ArmAngularState desState){
        desAngleDeg = desState.boomAngleDeg;
        desAngVelDegPerSec = desState.boomAnglularVel;
        desAngAccelDegPerSec = desState.boomAnglularAccel;

        //Apply command limits
        if(desAngleDeg > Constants.ARM_BOOM_MAX_ANGLE_DEG){
            desAngleDeg = Constants.ARM_BOOM_MAX_ANGLE_DEG;
            desAngVelDegPerSec = 0.0;
            desAngAccelDegPerSec = 0.0;
            isAngleLimited = true;
        } else if (desAngleDeg < Constants.ARM_BOOM_MIN_ANGLE_DEG){
            desAngleDeg = Constants.ARM_BOOM_MIN_ANGLE_DEG;
            desAngVelDegPerSec = 0.0;
            desAngAccelDegPerSec = 0.0;
            isAngleLimited = true;
        } else {
            isAngleLimited = false;
        }

    }    

    public void update(ArmAngularState act_in, boolean enabled){

        actAngleDeg = act_in.boomAngleDeg;
        actAngVelDegPerSec = act_in.boomAnglularVel;

        var motorCmdV = 0.0;

        // update PID Controller Constants
        var absErr = Math.abs(actAngleDeg - desAngleDeg);
        m_pid.setPID(kP.get(), kI.get(), kD.get());

        // Calculate Feed-Forward
        cmdFeedForward = Math.signum(desAngVelDegPerSec) * kS.get() + 
                        Math.cos(Units.degreesToRadians(desAngleDeg)) * kG.get() + 
                        desAngVelDegPerSec * kF.get();

        // Update feedback command
        cmdFeedBack = m_pid.calculate(actAngleDeg, desAngleDeg) * closedLoopGainSchedule.lookupVal(absErr);

        motorCmdV = cmdFeedForward + cmdFeedBack;

        // Send total command to motor;
        motorCtrl.setVoltageCmd(motorCmdV); 

        if(enabled){
            motorCtrl.setVoltageCmd(-1.0 * (cmdFeedForward + cmdFeedBack)); 
        } else {
            motorCtrl.setVoltageCmd(0.0);
        }
    }

}
