package frc.robot.Arm;

import edu.wpi.first.wpilibj.Solenoid;
import frc.Constants;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl.CANMotorCtrlType;

public class MotorControlBoom {


    WrapperedCANMotorCtrl motorCtrl = new WrapperedCANMotorCtrl("Boom", Constants.ARM_BOOM_MOTOR_CANID, CANMotorCtrlType.SPARK_MAX);

    Solenoid brakeSol;

    //TODO constructor

    public void setCmd(ArmState in){
        //todo - save off the right angles and velocities for this motor
    }    

    public void update(){

        //todo calculate brake state desired

        //todo calcualte feed forward for this segment (including gravity)

        //todo closed loop calcualtion

        //todo send stuff to the motor
        motorCtrl.setVoltageCmd(5.0); // TODO don't do this arm go zoooom

        //TODO send stuff to the brake
    }
}
