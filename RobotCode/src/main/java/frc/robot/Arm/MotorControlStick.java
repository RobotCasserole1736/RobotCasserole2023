package frc.robot.Arm;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
// import frc.Constants;

public class MotorControlStick {

    VictorSP motorCtrl = new VictorSP(5);
    public void setCmd(double in){
        //todo - save off the right angles and velocities for this motor
        motorCtrl.setVoltage(in);
    }

    public void update(double in){


        //todo calcualte feed forward for this segment (including gravity)

        //todo closed loop calcualtion

        //todo send stuff to the motor
        // motorCtrl.setVoltage(in); // TODO don't do this arm go zoooom
    }
}
