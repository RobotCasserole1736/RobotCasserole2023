package frc.robot.Arm;

import edu.wpi.first.math.util.Units;
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

    ArmState curMeasState;

    ArmPathPlanner pp;
    ArmManPosition mp;

    //TODO put offsets in for this
    WrapperedAbsoluteEncoder boomEncoder = new WrapperedAbsoluteEncoder(AbsoluteEncType.SRXEncoder, "Boom", Constants.ARM_BOOM_ENC_IDX, 0);
    WrapperedAbsoluteEncoder stickEncoder = new WrapperedAbsoluteEncoder(AbsoluteEncType.SRXEncoder, "Stick", Constants.ARM_STICK_ENC_IDX, 0);

    private ArmControl(){
        mb = new MotorControlBoom();
        ms = new MotorControlStick();
        pp = new ArmPathPlanner();
        mp = new ArmManPosition();
        curMeasState = new ArmState(0,0);
    }

    public void setInactive(){
        this.setOpCmds(0.0, 0.0, ArmNamedPosition.STOW, false);
    }

    public void setOpCmds(double desXVel, double desYVel, ArmNamedPosition posCmd, boolean posCmdActive){
        var manVelCmd = (desXVel != 0.0 || desYVel != 0.0);
        mp.setOpVelCmds(manVelCmd, desXVel, desYVel);
        pp.setCommand(posCmdActive, posCmd.pos);
    }

    public void update(){

        // Meas state and end effector position
        boomEncoder.update();
        stickEncoder.update();
        var boomAngleDeg = Units.radiansToDegrees(boomEncoder.getAngle_rad());
        var stickAngleDeg = Units.radiansToDegrees(stickEncoder.getAngle_rad());
        curMeasState = new ArmState(boomAngleDeg, stickAngleDeg); //TODO - add velocities here?
        ArmEndEffectorPos curMeasPos = ArmKinematics.forward(curMeasState);

        mp.update(curMeasPos);
        pp.update(curMeasPos);

        // Arbitrate to desired position from manual and pathplanned commands
        var curDesPosRaw = ArmPosCmdArbitration.arbitrate(mp.getCurDesPos(), pp.getCurDesPos());

        //Apply soft limits
        //var curDesPosLimited = ArmSoftLimits.applyLimit(curDesPosRaw);
        var curDesPosLimited = curDesPosRaw;

        // Apply kinematics to get linkge positions
        var curDesState = ArmKinematics.inverse(curDesPosLimited);

        // Send desired state to the motor control
        mb.setCmd(curDesState);
        mb.update(curMeasState);

        ms.setCmd(curDesState);
        ms.update(curMeasState);

        // Update telemetry
        ArmTelemetry.getInstance().setDesired(curDesPosLimited, curDesState);
        ArmTelemetry.getInstance().setMeasured(curMeasState);
    }
    
}
