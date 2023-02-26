package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.Calibration.Calibration;
import frc.lib.Faults.Fault;
import frc.lib.Signal.Annotations.Signal;

/**
 * Class to read input from whatever controller the operator is using, 
 * and convert it to operator commands
 */
public class OperatorInput {

    XboxController ctrl;

    @Signal
    boolean isConnected;

    // Manual up/down velocity command
    @Signal(units="mps")
    double curVerticalCmd;

    // Manual in/out velocity command
    @Signal(units="mps")
    double curHorizontalCmd;

    // Manual "offset up/down" position command
    @Signal(units="frac")
    double armVertOffsetCmd;


    // Automatic positioning commands
    @Signal
    boolean armLowPosCmd = false;
    @Signal
    boolean armMidPosCmd = false;
    @Signal
    boolean armHighPosCmd = false;
    @Signal
    boolean armStowPosCmd = false;

    // Manipulator grab/release
    @Signal
    boolean grabCmd = false;
    @Signal
    boolean releaseCmd = false;

    // Cone/Cube mode switch commands
    @Signal
    boolean switchToConeModeCmd = false;
    @Signal
    boolean switchToCubeModeCmd = false;

    // Deadband for controller sticks
    Calibration stickDb;

    // Maximum velocity for manual commands
    Calibration manMaxVel;

    // Fault for controller disconnected
    Fault disconFault = new Fault("Operator Controller", "Unplugged");

    String getName(int idx) {
        return "Operator Ctrl " + Integer.toString(idx) + " ";
    }

    public OperatorInput(int controllerIdx) {
        ctrl = new XboxController(controllerIdx);
        stickDb = new Calibration(getName(controllerIdx) + "Stick Deadband", "", 0.1);
        manMaxVel = new Calibration(getName(controllerIdx) + "Arm manual command max speed", "inches per second",
                12.0);
    }

    public void update() {

        isConnected = ctrl.isConnected();

        if (isConnected) {
            curVerticalCmd =  -1 * ctrl.getLeftY();
            curHorizontalCmd = -1.0 * ctrl.getRightY();

            curVerticalCmd = MathUtil.applyDeadband( curVerticalCmd,stickDb.get()) * Units.inchesToMeters(manMaxVel.get()); 
            curHorizontalCmd = MathUtil.applyDeadband( curHorizontalCmd,stickDb.get())  * Units.inchesToMeters(manMaxVel.get()); 

            armLowPosCmd = ctrl.getAButton();
            armMidPosCmd = ctrl.getBButton();
            armHighPosCmd = ctrl.getYButton();
            armStowPosCmd = ctrl.getXButton();

            armVertOffsetCmd = (ctrl.getLeftTriggerAxis() - ctrl.getRightTriggerAxis());

            grabCmd = ctrl.getLeftBumper();
            releaseCmd = ctrl.getRightBumper();

            switchToConeModeCmd = ctrl.getBackButton();
            switchToCubeModeCmd = ctrl.getStartButton();

        } else {
            // Controller Unplugged Defaults
            curVerticalCmd = 0.0;
            curHorizontalCmd = 0.0;
            armLowPosCmd = false;
            armMidPosCmd = false;
            armHighPosCmd = false;
            armStowPosCmd = false;
            armVertOffsetCmd = 0.0;
            grabCmd = false;
            releaseCmd = false;
        }

        disconFault.set(!isConnected);

    }

    public boolean posCmdActive(){ 
        return armLowPosCmd || armMidPosCmd || armHighPosCmd || armStowPosCmd;
    }

    public boolean manCmdActive(){
        return curVerticalCmd != 0.0 || curHorizontalCmd != 0.0;
    }
}
