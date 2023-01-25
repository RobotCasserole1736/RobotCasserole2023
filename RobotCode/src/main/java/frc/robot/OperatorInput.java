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
    
    XboxController operatorController;

    Calibration stickDeadband; 
    
    Calibration translateCmdScalar;

    @Signal(units="bool")
    boolean isConnected;

    @Signal(units="cmd")
    double curVerticalCmd;

    @Signal(units="cmd")
    double curHorizontalCmd;

    String getName(int idx){
        return "Driver Ctrl" + Integer.toString(idx) + " ";
    }

    public OperatorInput(int controllerIdx) {

        stickDeadband = new Calibration(getName(controllerIdx) + "StickDeadBand", "", 0.1);

    }

    public void update(){

        isConnected = operatorController.isConnected();

        if(isConnected){

            curVerticalCmd = .5 * operatorController.getLeftY();
            curHorizontalCmd = .5 * operatorController.getRightY();

            curVerticalCmd = MathUtil.applyDeadband( curVerticalCmd,stickDeadband.get()) * translateCmdScalar.get(); 
            curHorizontalCmd = MathUtil.applyDeadband( curHorizontalCmd,stickDeadband.get())  * translateCmdScalar.get();

        }else {
            curVerticalCmd = 0.0;
            curHorizontalCmd = 0.0;
        }


    }

}