package frc.robot.Arm;

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

    //TODO - add other control components for the arm

    //TODO put offsets in for this
    WrapperedAbsoluteEncoder boomEncoder = new WrapperedAbsoluteEncoder(AbsoluteEncType.SRXEncoder, "Boom", Constants.ARM_BOOM_ENC_IDX, 0);
    WrapperedAbsoluteEncoder stickEncoder = new WrapperedAbsoluteEncoder(AbsoluteEncType.SRXEncoder, "Stick", Constants.ARM_STICK_ENC_IDX, 0);

    private ArmControl(){
        mb = new MotorControlBoom();
        ms = new MotorControlStick();

    }

    public void update(){

        //TODO - add other updates for control compoennts for the arm

        mb.update();
        ms.update();

        // TODO - put meas/des things into telemetery, rather than tehse test values

        ArmState des = new ArmState();
        des.boomAngleDeg = 30;
        des.stickAngleDeg = -45;

        ArmState meas = new ArmState();
        meas.boomAngleDeg = 22;
        meas.stickAngleDeg = -57;

        ArmTelemetry.getInstance().setDesired(new ArmEndEffectorPos(0.7, 0.6), des);
        ArmTelemetry.getInstance().setMeasured(meas);
    }
    
}
