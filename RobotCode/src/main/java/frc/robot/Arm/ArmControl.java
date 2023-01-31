package frc.robot.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.Constants;
import frc.hardwareWrappers.AbsoluteEncoder.WrapperedAbsoluteEncoder;
import frc.hardwareWrappers.AbsoluteEncoder.WrapperedAbsoluteEncoder.AbsoluteEncType;
import frc.robot.ArmTelemetry;

public class ArmControl {

	private static ArmControl inst = null;
	public static synchronized ArmControl getInstance() {
		if(inst == null)
			inst = new ArmControl();
		return inst;
	}

    MotorControlBoom mb;
    MotorControlStick ms;

    ArmAngularState curMeasAngularStates;
    ArmEndEffectorState curDesState;

    ArmPathPlanner pp;
    ArmManPosition mp;

    ArmSoftLimits asl;

    //TODO put offsets in for this
    WrapperedAbsoluteEncoder boomEncoder = new WrapperedAbsoluteEncoder(AbsoluteEncType.SRXEncoder, "Boom", Constants.ARM_BOOM_ENC_IDX, 0);
    WrapperedAbsoluteEncoder stickEncoder = new WrapperedAbsoluteEncoder(AbsoluteEncType.SRXEncoder, "Stick", Constants.ARM_STICK_ENC_IDX, 0);

    private ArmControl(){
        mb = new MotorControlBoom();
        ms = new MotorControlStick();
        pp = new ArmPathPlanner();
        mp = new ArmManPosition();
        asl = new ArmSoftLimits();
        curMeasAngularStates = new ArmAngularState(0,0);
        curDesState = ArmNamedPosition.STOW.pos;
    }

    public void setInactive(){
        this.setOpCmds(0.0, 0.0, ArmNamedPosition.STOW, false);
    }

    public void setOpCmds(double desXVel, double desYVel, ArmNamedPosition posCmd, boolean posCmdActive){
        var manVelCmd = (desXVel != 0.0 || desYVel != 0.0);
        mp.setOpVelCmds(manVelCmd, desXVel, desYVel);
        pp.setCommand(posCmdActive, posCmd);
    }

    public void update(){

        // Meas state and end effector position
        boomEncoder.update();
        stickEncoder.update();
        var boomAngleDeg = Units.radiansToDegrees(boomEncoder.getAngle_rad());
        var stickAngleDeg = Units.radiansToDegrees(stickEncoder.getAngle_rad());
        curMeasAngularStates = new ArmAngularState(boomAngleDeg, stickAngleDeg); 
        ArmEndEffectorState curMeasState = ArmKinematics.forward(curMeasAngularStates);

        if(DriverStation.isDisabled()){
            curDesState = curMeasState;
        }

        curDesState = pp.update(curDesState);
        curDesState = mp.update(curDesState);

        //Apply soft limits
        //var curDesPosLimited = asl.applyLimit(curDesPosRaw);
        var curDesStateLimited = curDesState;

        // Apply kinematics to get linkge positions
        var curDesAngularStates = ArmKinematics.inverse(curDesStateLimited);

        // Send desired state to the motor control
        mb.setCmd(curDesAngularStates);
        mb.update(curMeasAngularStates);

        ms.setCmd(curDesAngularStates);
        ms.update(curMeasAngularStates);

        // Update telemetry
        ArmTelemetry.getInstance().setDesired(curDesStateLimited, curDesAngularStates);
        ArmTelemetry.getInstance().setMeasured(curMeasState, curMeasAngularStates);
    }
    
}
