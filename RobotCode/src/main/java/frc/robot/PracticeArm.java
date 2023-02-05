package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl.CANMotorCtrlType;

public class PracticeArm {

    XboxController op = new XboxController(1);
    VictorSP stickMotor = new VictorSP(5);
    WrapperedCANMotorCtrl boomMotor = new WrapperedCANMotorCtrl("Stick", 10, CANMotorCtrlType.SPARK_MAX);
    VictorSP intake = new VictorSP(6);
    double speedCmd;

    public void update(){



        stickMotor.set(-op.getRightY() * 0.5);
        boomMotor.setVoltageCmd(op.getLeftY() * 12.0 * 0.5);
        if (op.getBButton()){
            speedCmd = 1.0;
        }
        else if (op.getAButton()){
            speedCmd = -1.0;
           
        }
        else {
            speedCmd = 0;
        }
        intake.set(speedCmd);

    }
    
}
