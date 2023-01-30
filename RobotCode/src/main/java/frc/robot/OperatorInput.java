package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;

public class OperatorInput {

    XboxController ctrl;

    @Signal(units = "bool")
    boolean isConnected;

    @Signal(units="mps")
    double curVerticalCmd;

    @Signal(units="mps")
    double curHorizontalCmd;


    @Signal
    boolean armLowPosCmd = false;
    @Signal
    boolean armMidPosCmd = false;
    @Signal
    boolean armHighPosCmd = false;
    @Signal
    boolean armStowPosCmd = false;

    Calibration stickDb;
    Calibration manMaxVel;

    String getName(int idx) {
        return "Operator Ctrl " + Integer.toString(idx) + " ";
    }

    public OperatorInput(int controllerIdx) {
        ctrl = new XboxController(controllerIdx);
        stickDb = new Calibration(getName(controllerIdx) + "Stick Deadband", "", 0.1);
        manMaxVel = new Calibration(getName(controllerIdx) + "Arm manual command max speed", "inches per second",
                3.0);
    }

    public void update() {

        isConnected = ctrl.isConnected();

        if (isConnected) {
            curVerticalCmd =  ctrl.getLeftY();
            curHorizontalCmd = ctrl.getRightY();

            curVerticalCmd = MathUtil.applyDeadband( curVerticalCmd,stickDb.get()) * Units.inchesToMeters(manMaxVel.get()); 
            curHorizontalCmd = MathUtil.applyDeadband( curHorizontalCmd,stickDb.get())  * Units.inchesToMeters(manMaxVel.get()); 

            armLowPosCmd = ctrl.getAButton();
            armMidPosCmd = ctrl.getBButton();
            armHighPosCmd = ctrl.getYButton();
            armStowPosCmd = ctrl.getXButton();
        } else {
            // Controller Unplugged Defaults
            curVerticalCmd = 0.0;
            curHorizontalCmd = 0.0;
            armLowPosCmd = false;
            armMidPosCmd = false;
            armHighPosCmd = false;
            armStowPosCmd = false;
        }
    }

    public boolean posCmdActive(){ 
        return armLowPosCmd || armMidPosCmd || armHighPosCmd || armStowPosCmd;
    }

    public boolean manCmdActive(){
        return curVerticalCmd != 0.0 || curHorizontalCmd != 0.0;
    }
}
