package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl.CANMotorCtrlType;

public class PracticeArm {

    XboxController op = new XboxController(1);
    VictorSP stickMotor = new VictorSP(5);
    WrapperedCANMotorCtrl boomMotor = new WrapperedCANMotorCtrl("Stick", 10, CANMotorCtrlType.SPARK_MAX);

    public void update(){
        stickMotor.set(op.getRightY());
        boomMotor.setVoltageCmd(op.getLeftY() * 12.0);

    }
    
}
