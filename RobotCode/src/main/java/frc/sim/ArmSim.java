package frc.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.Constants;
import frc.hardwareWrappers.SimDeviceBanks;
import frc.hardwareWrappers.AbsoluteEncoder.Sim.SimAbsoluteEncoder;
import frc.hardwareWrappers.MotorCtrl.Sim.SimSmartMotor;
import frc.robot.ArmTelemetry;

public class ArmSim {

    private final double ARM_STICK_MASS = Units.lbsToKilograms(5);
    private final double ARM_BOOM_MASS = Units.lbsToKilograms(5);
    private final double ARM_END_EFF_MASS = Units.lbsToKilograms(10.0);

    private final double KIN_ROT_COEF_FRIC = 5.0;

    SimSmartMotor boomMotorCtrl;
    SimSmartMotor stickMotorCtrl;
    SimAbsoluteEncoder boomAbsEnc;
    SimAbsoluteEncoder stickAbsEnc;

    // Angles, in our defined reference system
    double curStickAngle_rad = 0.0;
    double curBoomAngle_rad = -Math.PI/2;

    // Speeds, in our reference system
    double curStickAngSpd_radpersec = 0.0;
    double curBoomAngSpd_radpersec = 0.0;

    // driven by one neo
    private final DCMotor m_boomGearbox = DCMotor.getNEO(1).withReduction(Constants.ARM_BOOM_GEAR_RATIO);
    private final DCMotor m_stickGearbox = DCMotor.getNEO(1).withReduction(Constants.ARM_STICK_GEAR_RATIO);

    public ArmSim(){
        boomMotorCtrl = (SimSmartMotor) SimDeviceBanks.getCANDevice(Constants.ARM_BOOM_MOTOR_CANID);
        stickMotorCtrl = (SimSmartMotor) SimDeviceBanks.getCANDevice(Constants.ARM_STICK_MOTOR_CANID);

        boomAbsEnc = (SimAbsoluteEncoder) SimDeviceBanks.getDIDevice(Constants.ARM_BOOM_ENC_IDX);
        stickAbsEnc = (SimAbsoluteEncoder) SimDeviceBanks.getDIDevice(Constants.ARM_STICK_ENC_IDX);
    }

    public void update(boolean isDisabled){
        var boomMotorTorque  = 0.0;
        var stickMotorTorque = 0.0;
        if(!isDisabled){
            var boomVoltage  = boomMotorCtrl.getAppliedVoltage_V();
            var stickVoltage = stickMotorCtrl.getAppliedVoltage_V();
    
            //Using the last speed, get our motor currents
            var boomCurrent = m_boomGearbox.getCurrent(curBoomAngSpd_radpersec, boomVoltage);
            var stickCurrent = m_stickGearbox.getCurrent(curStickAngSpd_radpersec, stickVoltage);
    
            //Using current, find the applied motor torque
            boomMotorTorque  = m_boomGearbox.getTorque(boomCurrent);
            stickMotorTorque = m_stickGearbox.getTorque(stickCurrent);
        } else {
            // Disabeld. Assume torques are 0 with coast mode
            // Brake mode would force voltage to zero.
        }



        // Stick Moment of Inertia - considered constant, as the sum of the MOI of the
        // stick itself, plus the end effecor (modeled as a point mass at distance stick_len from the pivot)
        var stick_moi = SingleJointedArmSim.estimateMOI(Constants.ARM_STICK_LENGTH, ARM_STICK_MASS) + 
                        ARM_END_EFF_MASS * Constants.ARM_STICK_LENGTH * Constants.ARM_STICK_LENGTH;

        // Boom Moment of Inertia - Variable. 
        // Simplified model - fixed MOI from the boom itself, plus point mass
        // at a distance from the center that varies by the stick angle.
        var stick_com_dist = Constants.ARM_BOOM_LENGTH + Math.cos(curStickAngle_rad) * Constants.ARM_STICK_LENGTH;
        var boom_moi = SingleJointedArmSim.estimateMOI(Constants.ARM_BOOM_LENGTH, ARM_BOOM_MASS) + 
                       (ARM_STICK_MASS + ARM_END_EFF_MASS) * stick_com_dist * stick_com_dist;


        //Gravitational Torques
        // Models gravity as acting on the components at fixed distances from centers of rotation
        // This definitely isn't perfect as it doesn't account for the total arm center of mass going off the axis
        var boomGravTorque = -9.81 * ARM_BOOM_MASS * Constants.ARM_BOOM_LENGTH/2.0 * Math.cos(curBoomAngle_rad) + //gravity acting on boom mass itself
                             -9.81 *  (ARM_STICK_MASS + ARM_END_EFF_MASS) * stick_com_dist * stick_com_dist * Math.cos(curBoomAngle_rad);

        var stickGravTorque =  -9.81 * (ARM_END_EFF_MASS) * Constants.ARM_BOOM_LENGTH * Math.cos(curBoomAngle_rad + curStickAngle_rad) +
                               -9.81 * (ARM_STICK_MASS) * Constants.ARM_BOOM_LENGTH/2.0 * Math.cos(curBoomAngle_rad + curStickAngle_rad);

        // Kinetic rotational friction torques
        var boomFricTorque = -1.0 * KIN_ROT_COEF_FRIC * curBoomAngSpd_radpersec;
        var stickFricTorque = -1.0 * KIN_ROT_COEF_FRIC * curStickAngSpd_radpersec;

        // Newton's second law
        // alpha = sum of torque / I

        var boomAlpha_radpersec2 = (boomGravTorque + boomMotorTorque + boomFricTorque) / boom_moi;
        var stickAlpha_radpersec2 = (stickGravTorque + stickMotorTorque + stickFricTorque) / stick_moi;

        // Integrate the silly way. Listen to tyler and oblarg sob quietly from afar, unsure of where these
        // voodoo doll like pains are coming from.
        curBoomAngSpd_radpersec += boomAlpha_radpersec2 * Constants.SIM_SAMPLE_RATE_SEC;
        curStickAngSpd_radpersec += stickAlpha_radpersec2 * Constants.SIM_SAMPLE_RATE_SEC;
        curBoomAngle_rad += curBoomAngSpd_radpersec * Constants.SIM_SAMPLE_RATE_SEC;
        curStickAngle_rad += curStickAngSpd_radpersec * Constants.SIM_SAMPLE_RATE_SEC;

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        boomAbsEnc.setRawAngle(curBoomAngle_rad);
        stickAbsEnc.setRawAngle(curStickAngle_rad);

        // Update the telemetry for the actual arm position
        var boomAngleDeg = Units.radiansToDegrees(curBoomAngle_rad);
        var stickAngleDeg = Units.radiansToDegrees(curStickAngle_rad);
        ArmTelemetry.getInstance().setActual(boomAngleDeg, stickAngleDeg);
    }
        
}
