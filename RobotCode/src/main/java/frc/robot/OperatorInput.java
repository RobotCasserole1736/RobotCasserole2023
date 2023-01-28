package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;

public class OperatorInput {

    @Signal(units="bool")
    boolean isConnected;
    @Signal(units="cmd")
    double boomMotor;
    @Signal(units="cmd")
    double stickMotor;
    
    XboxController operatorController;
    Calibration stickDeadband;

    String getName(int idx){
        return "Driver Ctrl " + Integer.toString(idx) + " ";
    }

    public OperatorInput(int controllerIdx){
        operatorController = new XboxController(controllerIdx);
        stickDeadband = new Calibration(getName(controllerIdx) + "StickDeadBand", "", 0.1);
    }
    
    public void update(){

        isConnected = operatorController.isConnected();

    if(isConnected){
        boomMotor = -1.0 * operatorController.getLeftY();
        stickMotor = -1.0 * operatorController.getRightY();

    } else {
            //Controller Unplugged Defaults
            boomMotor = 0.0;
            stickMotor = 0.0;
        }
    }
}
