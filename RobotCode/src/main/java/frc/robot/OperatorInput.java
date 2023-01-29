package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;

public class OperatorInput {

    @Signal(units="bool")
    boolean isConnected;
    @Signal(units="cmd")
    double boomMotorCmd;
    @Signal(units="cmd")
    double stickMotorCmd;
    
    XboxController operatorController;
    Calibration stickDeadband;
    Calibration boomMotorCmdScalar;
    Calibration stickMotorCmdScalar;

    String getName(int idx){
        return "Driver Ctrl " + Integer.toString(idx) + " ";
    }

    public OperatorInput(int controllerIdx){
        operatorController = new XboxController(controllerIdx);
        stickDeadband = new Calibration(getName(controllerIdx) + "StickDeadBand", "", 0.1);
        boomMotorCmdScalar = new Calibration(getName(controllerIdx) + "boomMotorCmdScalar", "", 0.8);
        stickMotorCmdScalar = new Calibration(getName(controllerIdx) + "stickMotorCmdScalar", "", 0.8);
    }
    
    public void update(){

        isConnected = operatorController.isConnected();

        if(isConnected){
            boomMotorCmd = -1.0 * operatorController.getLeftY();
            stickMotorCmd = -1.0 * operatorController.getRightY();
            
            boomMotorCmd = MathUtil.applyDeadband( boomMotorCmd,stickDeadband.get()) * boomMotorCmdScalar.get(); 
            stickMotorCmd = MathUtil.applyDeadband( stickMotorCmd,stickDeadband.get())  * stickMotorCmdScalar.get();
        } else {
                //Controller Unplugged Defaults
                boomMotorCmd = 0.0;
                stickMotorCmd = 0.0;
            }
        }
}
