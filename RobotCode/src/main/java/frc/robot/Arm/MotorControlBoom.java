package frc.robot.Arm;

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

    Solenoid brakeSol;

    @Signal
    double desAngleDeg;
    @Signal
    double actAngleDeg;

    //TODO constructor

    public void setCmd(ArmAngularState in){
        //todo - save off the right angles and velocities for this motor
        desAngleDeg = in.boomAngleDeg;
    }    

    public void update(ArmAngularState act_in){

        //todo calculate brake state desired

        //todo calcualte feed forward for this segment (including gravity)

        //todo closed loop calcualtion that's better than this
        actAngleDeg = act_in.boomAngleDeg;
        var err =  desAngleDeg - actAngleDeg;
        var cmd = 12.0/5.0 * err;

        //todo send stuff to the motor
        motorCtrl.setVoltageCmd(cmd); // TODO don't do this arm go zoooom

        //TODO send stuff to the brake
    }
}
