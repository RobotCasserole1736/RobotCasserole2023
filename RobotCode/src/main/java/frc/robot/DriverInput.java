package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
    Calibration armExtenedLimitFactor;
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

    // Auxilary intake/eject commands
    @Signal(units="bool")
    boolean clawEject;
    @Signal(units="bool")
    boolean clawIntake; 


    // Auto-drive commands
    @Signal(units="bool")
    boolean spinMoveCmd;
    @Signal(units="bool")
    boolean driveToCenterCmd;

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
        rotSlewRate = new Calibration(getName(controllerIdx) + "rotSlewRate", "", Constants.MAX_ROTATE_ACCEL_RAD_PER_SEC_2*0.5);
        sideToSideSlewRate = new Calibration(getName(controllerIdx) + "sideToSideSlewRate", "", Constants.MAX_TRANSLATE_ACCEL_MPS2*0.75);

        translateCmdScalar = new Calibration(getName(controllerIdx) + "translateCmdScalar", "", 1.0);
        rotateCmdScalar = new Calibration(getName(controllerIdx) + "rotateCmdScalar", "", 1.0);

        armExtenedLimitFactor = new Calibration(getName(controllerIdx) + "armExtendedSpdLimitFactor", "frac", 0.75);
        armExtenedFarLimitFactor = new Calibration(getName(controllerIdx) + "armExtendedSpdLimitFactor", "frac", 0.5);
        armExtenedReallyFarLimitFactor = new Calibration(getName(controllerIdx) + "armExtendedSpdLimitFactor", "frac", 0.25);

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

            //Convert to raw meters per second, using drivetrain max theoretical speed
            var curFwdRevSpdRaw = curFwdRevCmd * Constants.MAX_FWD_REV_SPEED_MPS;
            var curRotSpdRaw = curRotCmd * Constants.MAX_ROTATE_SPEED_RAD_PER_SEC;
            var curSideToSideSpdRaw = curSideToSideCmd * Constants.MAX_FWD_REV_SPEED_MPS;

            //Scale back the speed command if the arm is extended too far.
            if(ArmControl.getInstance().isExtended()){
                var factor = armExtenedLimitFactor.get();
                curFwdRevSpdRaw *= factor;
                curRotSpdRaw *= factor;
                curSideToSideSpdRaw *= factor;
            }
            else if(ArmControl.getInstance().isExtendedFar()){
                var factor = armExtenedFarLimitFactor.get();
                curFwdRevSpdRaw *= factor;
                curRotSpdRaw *= factor;
                curSideToSideSpdRaw *= factor;
            }
            else if(ArmControl.getInstance().isExtendedReallyFar()){
                var factor = armExtenedReallyFarLimitFactor.get();
                curFwdRevSpdRaw *= factor;
                curRotSpdRaw *= factor;
                curSideToSideSpdRaw *= factor;
            }

            // Slew rate limit the command to prevent jerky movement
            fwdRevSpdCmd = fwdRevSlewLimiter.calculate(curFwdRevSpdRaw );
            rotSpdCmd = rotSlewLimiter.calculate(curRotSpdRaw);
            sideToSideSpdCmd = sideToSideSlewLimiter.calculate(curSideToSideSpdRaw);
                
            // Read in other drivetrain controls
            robotRelative = driverController.getRightBumper();
            braceCmd = driverController.getLeftBumper();
            resetOdometry = resetOdoDbnc.calculate(driverController.getAButton());

            // Read in Auto-drive commands
            spinMoveCmd = driverController.getBButton();
            driveToCenterCmd = driverController.getXButton();

            // Read in claw intake/eject commands
            clawEject = driverController.getRightTriggerAxis() > .75;
            clawIntake = driverController.getLeftTriggerAxis() > .75;

            //If both are pulled, make sure we don't do both at the same time
            if(clawEject && clawIntake) {
                clawEject = false;
                clawIntake = false;
            }

 
        } else {
            //Controller Unplugged Defaults
            fwdRevSpdCmd = 0.0;
            rotSpdCmd = 0.0; 
            sideToSideSpdCmd = 0.0; 
            robotRelative = false;
            resetOdometry = false;
            spinMoveCmd = false;
            driveToCenterCmd = false;
            clawEject = false;
            clawIntake = false;
            braceCmd = false;
        }

        disconFault.set(!isConnected);

        
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

    public boolean getRelease(){
        return clawEject;
    }

    public boolean getGrab(){
        return clawIntake;
    }

    public boolean getSpinMoveCmd(){
        return spinMoveCmd;
    }

    public boolean getDriveToCenterCmd(){
        return driveToCenterCmd;
    }

    public boolean getBracePositionCmd(){
        return braceCmd;
    }

}