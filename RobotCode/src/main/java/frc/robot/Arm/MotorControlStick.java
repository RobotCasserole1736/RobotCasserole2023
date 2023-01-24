package frc.robot.Arm;

import frc.Constants;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl.CANMotorCtrlType;

public class MotorControlStick {

    WrapperedCANMotorCtrl motorCtrl = new WrapperedCANMotorCtrl("Stick", Constants.ARM_STICK_MOTOR_CANID, CANMotorCtrlType.SPARK_MAX);

    public void setCmd(ArmState in){
        //todo - save off the right angles and velocities for this motor
    }

    public void update(){


        //todo calcualte feed forward for this segment (including gravity)

        //todo closed loop calcualtion

        //todo send stuff to the motor
        motorCtrl.setVoltageCmd(5.0); // TODO don't do this arm go zoooom

    }
    
}
