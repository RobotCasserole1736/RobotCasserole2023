package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.XboxController;
import frc.Constants;
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



    public OperatorInput(int controllerIdx){

        operatorController = new XboxController(controllerIdx);

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
