package frc.robot.Arm;

import java.sql.Driver;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.Constants;
import frc.hardwareWrappers.AbsoluteEncoder.WrapperedAbsoluteEncoder;
import frc.hardwareWrappers.AbsoluteEncoder.WrapperedAbsoluteEncoder.AbsoluteEncType;
import frc.lib.Util.FunctionGenerator;
import frc.robot.ArmTelemetry;

public class ArmControl {

	private static ArmControl inst = null;
	public static synchronized ArmControl getInstance() {
		if(inst == null)
			inst = new ArmControl();
		return inst;
	}

    boolean releaseBrakeCmd;

    MotorControlBoom mb;
    MotorControlStick ms;

    ArmAngularState curMeasAngularStates;
    ArmEndEffectorState curDesState;
    ArmEndEffectorState prevDesState;

    ArmPathPlanner pp;
    ArmManPosition mp;
    ArmConePlaceOffset cpo;

    ArmSoftLimits asl;

    WrapperedAbsoluteEncoder boomEncoder = new WrapperedAbsoluteEncoder(AbsoluteEncType.RevThroughBore, "Boom", Constants.ARM_BOOM_ENC_IDX, Constants.ARM_BOOM_ENCODER_MOUNT_OFFSET_RAD, false);
    WrapperedAbsoluteEncoder stickEncoder = new WrapperedAbsoluteEncoder(AbsoluteEncType.RevThroughBore, "Stick", Constants.ARM_STICK_ENC_IDX, Constants.ARM_STICK_ENCODER_MOUNT_OFFSET_RAD, true);

    // Test mode tools
    // These help us inject specific waveforms into swerve modules to calibrate and test them.
    FunctionGenerator boomFG;
    FunctionGenerator stickFG;

    private ArmControl(){
        mb = new MotorControlBoom();
        ms = new MotorControlStick();
        pp = new ArmPathPlanner();
        mp = new ArmManPosition();
        cpo = new ArmConePlaceOffset();
        asl = new ArmSoftLimits();
        curMeasAngularStates = new ArmAngularState(0,0);
        curDesState = ArmNamedPosition.STOW.get();
        prevDesState = ArmNamedPosition.STOW.get();

        boomFG = new FunctionGenerator("arm_boom", "deg");
        stickFG = new FunctionGenerator("arm_stick", "deg");

    }

    public void setInactive(){
        this.setOpCmds(0.0, 0.0, ArmNamedPosition.STOW, false, 0.0);
    }

    public void setOpCmds(double desXVel, double desYVel, ArmNamedPosition posCmd, boolean posCmdActive, double vertOffsetCmd){
        var manVelCmd = !posCmdActive;
        mp.setOpVelCmds(manVelCmd, desXVel, desYVel);
        pp.setCommand(posCmdActive, posCmd);
        cpo.setCmd(vertOffsetCmd);
    }

    private ArmEndEffectorState updateMeasState(){
        // Meas state and end effector position
        boomEncoder.update();
        stickEncoder.update();
        boomEncoder.isFaulted();
        stickEncoder.isFaulted();
        var boomAngleDeg = Units.radiansToDegrees(boomEncoder.getAngle_rad());
        var stickAngleDeg = Units.radiansToDegrees(stickEncoder.getAngle_rad());
        curMeasAngularStates = new ArmAngularState(boomAngleDeg, stickAngleDeg);      
        return ArmKinematics.forward(curMeasAngularStates);   
    }

    public void update(){

        ArmEndEffectorState curMeasState = updateMeasState();

        curDesState = new ArmEndEffectorState();
        ArmEndEffectorState curDesStateWithOffset;

        if(DriverStation.isDisabled()){
            // While disabled, by default, we just maintain measured state
            curDesState.x = curMeasState.x;
            curDesState.y = curMeasState.y;
            curDesState.reflexFrac = curMeasState.reflexFrac;
        } else {
            // While endabled, by default, the next desired state
            // is just the position of the desired state position, with zero velocity
            curDesState.x = prevDesState.x;
            curDesState.y = prevDesState.y;
            curDesState.reflexFrac = prevDesState.reflexFrac;
        }

        // Allow the brakes to be released in disabled for easy manipulation in the pit
        if(DriverStation.isDisabled() && RobotController.getUserButton()){
            mb.setBrakeMode(false);
            ms.setBrakeMode(false);
        } else {
            mb.setBrakeMode(true);
            ms.setBrakeMode(true);
        }

        // Allow the path planner top (optinoally) modify the desired state
        curDesState = pp.update(curDesState);

        // Allow the manual motion module to (optionally) modify the desired state
        curDesState = mp.update(curDesState);

        // Allow the offset module to (optionally) modify the desired state
        curDesStateWithOffset = cpo.update(curDesState);

        //Apply soft limits
        var curDesStateLimited = asl.applyLimit(curDesStateWithOffset);
        //var curDesStateLimited = curDesStateWithOffset;

        // Apply kinematics to get linkge positions
        var curDesAngularStates = ArmKinematics.inverse(curDesStateLimited);

        // Send desired state to the motor control
        mb.setCmd(curDesAngularStates);
        if(isFaulted() == true){
            mb.update(curMeasAngularStates, false);
        } else {
            mb.update(curMeasAngularStates, true);
        }

        ms.setCmd(curDesAngularStates);
        if(isFaulted() == true){
            ms.update(curMeasAngularStates, false);
        } else {
            ms.update(curMeasAngularStates, true);
        }


        // Update telemetry
        ArmTelemetry.getInstance().setDesired(curDesStateWithOffset, curDesAngularStates);
        ArmTelemetry.getInstance().setMeasured(curMeasState, curMeasAngularStates);

        // Save previous
        prevDesState = curDesState;
    }

    private boolean isFaulted() {
        return false;
    }

    //Test-mode only for tuning
    public void testUpdate(){

        ArmEndEffectorState curMeasState = updateMeasState();

        var testDesState = new ArmAngularState();

        testDesState.boomAngleDeg = boomFG.getValue();
        testDesState.boomAnglularVel = boomFG.getValDeriv();
        testDesState.stickAngleDeg = stickFG.getValue();
        testDesState.stickAngularVel = stickFG.getValDeriv();

        mb.setCmd(testDesState);
        mb.update(curMeasAngularStates, boomFG.isEnabled());

        ms.setCmd(testDesState);
        ms.update(curMeasAngularStates,stickFG.isEnabled());

        var curTestState = ArmKinematics.forward(testDesState);

        // Update telemetry
        ArmTelemetry.getInstance().setDesired(curTestState, testDesState);
        ArmTelemetry.getInstance().setMeasured(curMeasState, curMeasAngularStates);
        
    }

        public boolean isPathPlanning(){
        return pp.motionActive;
    }

    public boolean isExtended(){
        return curDesState.x > Constants.WHEEL_BASE_HALF_LENGTH_M;
    }

    public boolean isSoftLimited(){
        return asl.isLimited() || mb.isAngleLimited || ms.isAngleLimited;
    }
    
}
