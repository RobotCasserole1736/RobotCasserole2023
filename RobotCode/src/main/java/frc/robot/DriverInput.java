package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.Constants;
import frc.lib.Calibration.Calibration;
import frc.lib.Faults.Fault;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.Arm.ArmControl;

/**
 * Class to read whatever physical controller the driver has
 * and convert it into actual commands from the driver
 */
public class DriverInput {
    
    // the controller itself
    XboxController driverController;

    // Slew rate limits so the driver's commands
    // to the drivetrain don't change too fast
    SlewRateLimiter fwdRevSlewLimiter;
    SlewRateLimiter rotSlewLimiter;
    SlewRateLimiter sideToSideSlewLimiter;

    // Calibrations impacting driver feel
    Calibration stickDeadband;
    Calibration fwdRevSlewRate;
    Calibration rotSlewRate;
    Calibration sideToSideSlewRate;
    Calibration translateCmdScalar;
    Calibration rotateCmdScalar;
    Calibration armExtenedFarLimitFactor;
    Calibration armExtenedReallyFarLimitFactor;
    
    // Drivetrain movement commands (from joysticks)
    @Signal(units="cmd")
    double curFwdRevCmd;
    @Signal(units="cmd")
    double curRotCmd;
    @Signal(units="cmd")
    double curSideToSideCmd;

    // Drivetrain movement commands (in physical units)
    @Signal (units="mps")
    double fwdRevSpdCmd;
    @Signal (units="radPerSec")
    double rotSpdCmd;
    @Signal (units="mps")
    double sideToSideSpdCmd;

    // Switch the robot into robot relative mode
    @Signal(units="bool")
    boolean robotRelative;

    // Command the wheels to assume a brace position
    @Signal(units="bool")
    boolean braceCmd;

    // True if the drive wants auto turn, false otherwise
    @Signal(units="bool")
    public boolean autoTurn;


    // Auto-drive commands
    @Signal(units="bool")
    public boolean adLeft;
    @Signal(units="bool")
    public boolean adCenter;
    @Signal(units="bool")
    public boolean adRight;
    @Signal(units="bool")
    public boolean adLeftModifier;
    @Signal(units="bool")
    public boolean adRightModifier; 

    // Press-and-hold to reset odometry to be pointing downfield.
    @Signal(units="bool")
    boolean resetOdometry;
    Debouncer resetOdoDbnc = new Debouncer(0.5, DebounceType.kRising);

    @Signal(units="bool")
    boolean isConnected;

    Fault disconFault = new Fault("Driver Controller", "Unplugged");

    String getName(int idx){
        return "Driver Ctrl " + Integer.toString(idx) + " ";
    }

    public DriverInput(int controllerIdx){

        driverController = new XboxController(controllerIdx);

        stickDeadband = new Calibration(getName(controllerIdx) + "StickDeadBand", "", 0.1);

        fwdRevSlewRate = new Calibration(getName(controllerIdx) + "fwdRevSlewRate_", "", Constants.MAX_TRANSLATE_ACCEL_MPS2*0.75);
        sideToSideSlewRate = new Calibration(getName(controllerIdx) + "sideToSideSlewRate", "", Constants.MAX_TRANSLATE_ACCEL_MPS2*0.75);
        rotSlewRate = new Calibration(getName(controllerIdx) + "rotSlewRate", "", Constants.MAX_ROTATE_ACCEL_RAD_PER_SEC_2*0.4);

        translateCmdScalar = new Calibration(getName(controllerIdx) + "translateCmdScalar", "", 1.0);
        rotateCmdScalar = new Calibration(getName(controllerIdx) + "rotateCmdScalar", "", 1.0);

        fwdRevSlewLimiter = new SlewRateLimiter(fwdRevSlewRate.get());
        rotSlewLimiter = new SlewRateLimiter(rotSlewRate.get());
        sideToSideSlewLimiter = new SlewRateLimiter(sideToSideSlewRate.get());
    }

    public void update(){

        isConnected = driverController.isConnected();

        if(isConnected){
            
            //Read in raw -1 to 1 commands from the joysticks, re-orient
            curFwdRevCmd = -1.0 * driverController.getLeftY();
            curRotCmd = -1.0 * driverController.getRightX();
            curSideToSideCmd = -1.0 * driverController.getLeftX();

            //Apply a deadband, then a scale factor to make the robot easier to drive in certain directions
            curFwdRevCmd = MathUtil.applyDeadband( curFwdRevCmd,stickDeadband.get()) * translateCmdScalar.get(); 
            curRotCmd = MathUtil.applyDeadband( curRotCmd,stickDeadband.get())  * rotateCmdScalar.get();
            curSideToSideCmd = MathUtil.applyDeadband( curSideToSideCmd,stickDeadband.get())  * translateCmdScalar.get();

            //Convert to raw meters per second, using drivetrain max theoretical speed plus some margin to ensure we can actually achieve max speed
            var curFwdRevSpdRaw = curFwdRevCmd * Constants.MAX_FWD_REV_SPEED_MPS * 0.75;
            var curRotSpdRaw = curRotCmd * Constants.MAX_ROTATE_SPEED_RAD_PER_SEC * 0.75;
            var curSideToSideSpdRaw = curSideToSideCmd * Constants.MAX_FWD_REV_SPEED_MPS * 0.75;

            //Scale back the speed command if the arm is extended too far.
            var factor = ArmControl.getInstance().speedLimitFactorCalc();
            curFwdRevSpdRaw *= factor;
            curRotSpdRaw *= factor;
            curSideToSideSpdRaw *= factor;

            // Slew rate limit the command to prevent jerky movement
            fwdRevSpdCmd = fwdRevSlewLimiter.calculate(curFwdRevSpdRaw );
            rotSpdCmd = rotSlewLimiter.calculate(curRotSpdRaw);
            sideToSideSpdCmd = sideToSideSlewLimiter.calculate(curSideToSideSpdRaw);
                
            // Read in other drivetrain controls
            robotRelative = driverController.getRightBumper();
            braceCmd = driverController.getLeftBumper();
            resetOdometry = resetOdoDbnc.calculate(driverController.getAButton());

            // Read in Auto-drive commands
            adLeft = driverController.getXButton();
            adCenter = driverController.getYButton();
            adRight = driverController.getBButton();
            adLeftModifier = driverController.getLeftTriggerAxis() > .75;
            adRightModifier = driverController.getRightTriggerAxis() > .75;
            autoTurn = driverController.getRightStickButton();

 
        } else {
            //Controller Unplugged Defaults
            fwdRevSpdCmd = 0.0;
            rotSpdCmd = 0.0; 
            sideToSideSpdCmd = 0.0; 
            robotRelative = false;
            resetOdometry = false;
            adLeft = false;
            adCenter = false;
            adRight = false;
            adLeftModifier = false;
            adRightModifier = false;
            braceCmd = false;
            autoTurn = false;
        }

        disconFault.set(!isConnected && DriverStation.isDSAttached());

        
        //Slew rate limiters don't have the ability to change the slew rate on the fly,
        // so we'll just recreate them as new objects whenever the cal value is changed.
        if(fwdRevSlewRate.isChanged() ||
           rotSlewRate.isChanged() ||
           sideToSideSlewRate.isChanged()) {
                fwdRevSlewRate.acknowledgeValUpdate();
                rotSlewRate.acknowledgeValUpdate();
                sideToSideSlewRate.acknowledgeValUpdate();
                fwdRevSlewLimiter = new SlewRateLimiter(fwdRevSlewRate.get());
                rotSlewLimiter = new SlewRateLimiter(rotSlewRate.get());
                sideToSideSlewLimiter = new SlewRateLimiter(sideToSideSlewRate.get());
        }
               
           
        
    }

    /**
     * Gets the driver command for fwd/rev
     * @return 
     */
    public double getFwdRevCmd_mps(){
        return fwdRevSpdCmd;
    }

    /**
     * Gets the driver command for rotate
     * @return 
     */
    public double getRotateCmd_rps(){
        return rotSpdCmd;
    }

    /**
     * Gets the driver command for side to side strafing
     * @return 
     */
    public double getSideToSideCmd_mps(){
        return sideToSideSpdCmd;
    }


    /**
     * @return True if the driver wants the speed commands to be relative to the robot's reference frame,
     * False if the commands should be realtive to the field reference frame
     */
    public boolean getRobotRelative(){
        return robotRelative;
    }

    /**
     * @return True if the driver wants to reset odometry to say the the robot is pointed
     * downfield, false otherwise
     */
    public boolean getOdoResetCmd(){
        return resetOdometry;
    }

    public boolean getBracePositionCmd(){
        return braceCmd;
    }

}