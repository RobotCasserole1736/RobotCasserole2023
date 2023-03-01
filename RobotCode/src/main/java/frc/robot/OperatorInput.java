package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

    final double MAX_MAN_VEL_MPS = Units.inchesToMeters(12.0);

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
    boolean armVertOffsetTrig;

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
    @Signal
    boolean armFloorTippedConePosCmd = false;
    @Signal
    boolean armShelfPosCmd = false;

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

    SlewRateLimiter horizCmdSlewRate = new SlewRateLimiter(MAX_MAN_VEL_MPS * 3);
    SlewRateLimiter vertCmdSlewRate = new SlewRateLimiter(MAX_MAN_VEL_MPS * 2);
    SlewRateLimiter horizOffsetSlewRate = new SlewRateLimiter(1.0);

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
        stickDb = new Calibration(getName(controllerIdx) + "Stick Deadband", "", 0.15);
    }

    public void update() {

        isConnected = ctrl.isConnected();

        if (isConnected) {
            //Vertical command is always just vertical command
            curVerticalCmd =  MathUtil.applyDeadband( -1.0 * ctrl.getLeftY(),stickDb.get());
            curVerticalCmd = vertCmdSlewRate.calculate( curVerticalCmd * MAX_MAN_VEL_MPS); 

            armLowPosCmd = ctrl.getAButton();
            armMidPosCmd = ctrl.getBButton();
            armHighPosCmd = ctrl.getYButton();
            armStowPosCmd = ctrl.getXButton();
            armFloorTippedConePosCmd = povDown();
            armShelfPosCmd = povUP();

            armVertOffsetTrig = ctrl.getRightTriggerAxis() > 0.5;

            var horizCmdRaw = MathUtil.applyDeadband( ctrl.getRightY(),stickDb.get());
            if(armVertOffsetTrig == true) {
                // Use horizontal command as an offset, zero velocity
                armVertOffsetCmd = horizOffsetSlewRate.calculate(-1 * horizCmdRaw * MAX_MAN_VEL_MPS);
                curHorizontalCmd = horizCmdSlewRate.calculate(0.0);
            } else {
                // Use horizontal command as a velocity command, zero offset
                armVertOffsetCmd = horizOffsetSlewRate.calculate(0.0);
                curHorizontalCmd = horizCmdSlewRate.calculate(horizCmdRaw);
            }

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
            armShelfPosCmd = false;
            armFloorTippedConePosCmd = false;
            armVertOffsetTrig = false;
            grabCmd = false;
            releaseCmd = false;
        }

        disconFault.set(!isConnected);

    }

    public boolean posCmdActive(){ 
        return armLowPosCmd || armMidPosCmd || armHighPosCmd || armStowPosCmd || armFloorTippedConePosCmd || armShelfPosCmd;
    }

    public boolean manCmdActive(){
        return curVerticalCmd != 0.0 || curHorizontalCmd != 0.0;
    }

    private boolean povDown(){
        return (ctrl.getPOV() != -1 && ctrl.getPOV() <= 225 && ctrl.getPOV() >= 135);
    }

    private boolean povUP(){
        return (ctrl.getPOV() != -1 && (ctrl.getPOV() >= 315 || ctrl.getPOV() <= 45));
    }
}
