package frc.robot.Arm;

import edu.wpi.first.wpilibj.Solenoid;
import frc.Constants;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl.CANMotorCtrlType;
import frc.lib.Signal.Annotations.Signal;

public class MotorControlBoom {


    WrapperedCANMotorCtrl motorCtrl = new WrapperedCANMotorCtrl("Boom", Constants.ARM_BOOM_MOTOR_CANID, CANMotorCtrlType.SPARK_MAX);

    Solenoid brakeSol;

    @Signal
    double desAngleDeg;
    @Signal
    double actAngleDeg;

    //TODO constructor

    public void setCmd(ArmState in){
        //todo - save off the right angles and velocities for this motor
        desAngleDeg = in.boomAngleDeg;
    }    

    public void update(ArmState act_in){

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
