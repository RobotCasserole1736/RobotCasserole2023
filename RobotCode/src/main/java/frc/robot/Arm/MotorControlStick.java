package frc.robot.Arm;

import frc.Constants;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl.CANMotorCtrlType;
import frc.lib.Signal.Annotations.Signal;

public class MotorControlStick {

    WrapperedCANMotorCtrl motorCtrl = new WrapperedCANMotorCtrl("Stick", Constants.ARM_STICK_MOTOR_CANID, CANMotorCtrlType.SPARK_MAX);


    @Signal
    double desAngleDeg;
    @Signal
    double actAngleDeg;

    public void setCmd(ArmState in){
        //todo - save off the right angles and velocities for this motor
        desAngleDeg = in.stickAngleDeg;
    }

    public void update(ArmState act_in){


        //todo calcualte feed forward for this segment (including gravity)

        //todo closed loop calcualtion that's better than this
        actAngleDeg = act_in.stickAngleDeg;
        var err =  desAngleDeg - actAngleDeg;
        var cmd = 12.0/5.0 * err;

        //todo send stuff to the motor
        motorCtrl.setVoltageCmd(cmd); // TODO don't do this arm go zoooom

    }
    
}
