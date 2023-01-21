package frc.robot.Arm;

import edu.wpi.first.wpilibj.Solenoid;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;

public class MotorControlBoom {


    WrapperedCANMotorCtrl motorCtrl;

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

        //TODO send stuff to the brake
    }
}
