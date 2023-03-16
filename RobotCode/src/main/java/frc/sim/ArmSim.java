package frc.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import frc.Constants;
import frc.hardwareWrappers.SimDeviceBanks;
import frc.hardwareWrappers.AbsoluteEncoder.Sim.SimAbsoluteEncoder;
import frc.hardwareWrappers.MotorCtrl.Sim.SimSmartMotor;
import frc.robot.ArmTelemetry;

public class ArmSim {

    private final double ARM_STICK_MASS = Units.lbsToKilograms(3.0);
    private final double ARM_BOOM_MASS = Units.lbsToKilograms(3.0);
    private final double ARM_END_EFF_MASS = Units.lbsToKilograms(3.0);

    private final double KIN_ROT_COEF_FRIC = 1.0;

    SimSmartMotor boomMotorCtrl;
    SimSmartMotor stickMotorCtrl;
    SimAbsoluteEncoder boomAbsEnc;
    SimAbsoluteEncoder stickAbsEnc;

    // Angles, in our defined reference system
    double curStickAngle_rad = Units.degreesToRadians(160.0);
    double curBoomAngle_rad= Units.degreesToRadians(-120.0);

    // Speeds, in our reference system
    double curStickAngSpd_radpersec = 0.0;
    double curBoomAngSpd_radpersec = 0.0;

    // driven by one neo each
    private final DCMotor m_boomGearbox = DCMotor.getNEO(1).withReduction(Constants.ARM_BOOM_GEAR_RATIO);
    private final DCMotor m_stickGearbox = DCMotor.getNEO(1).withReduction(Constants.ARM_STICK_GEAR_RATIO);

    // With a brake ont he boom
    private final SolenoidSim m_boomBrakeSol = new SolenoidSim(PneumaticsModuleType.CTREPCM, Constants.ARM_BOOM_BRAKE_SOLENOID);

    public ArmSim(){
        boomMotorCtrl = (SimSmartMotor) SimDeviceBanks.getCANDevice(Constants.ARM_BOOM_MOTOR_CANID);
        stickMotorCtrl = (SimSmartMotor) SimDeviceBanks.getCANDevice(Constants.ARM_STICK_MOTOR_CANID);

        boomAbsEnc = (SimAbsoluteEncoder) SimDeviceBanks.getDIDevice(Constants.ARM_BOOM_ENC_IDX);
        stickAbsEnc = (SimAbsoluteEncoder) SimDeviceBanks.getDIDevice(Constants.ARM_STICK_ENC_IDX);

    }

    public void update(boolean isDisabled){

        var boomVoltage = 0.0;
        var stickVoltage = 0.0;

        if(!isDisabled){
            boomVoltage  = boomMotorCtrl.getAppliedVoltage_V() * -1.0; //it's mechanically inverted
            stickVoltage = stickMotorCtrl.getAppliedVoltage_V() * -1.0; //it's mechanically inverted
        } 

        //Using the last speed, get our motor currents
        var boomCurrent  = boomMotorCtrl.sim_isCoasting()  ? 0 : m_boomGearbox.getCurrent(curBoomAngSpd_radpersec, boomVoltage);
        var stickCurrent = stickMotorCtrl.sim_isCoasting() ? 0 : m_stickGearbox.getCurrent(curStickAngSpd_radpersec, stickVoltage);

        //Using current, find the applied motor torque
        var boomMotorTorque  = m_boomGearbox.getTorque(boomCurrent);
        var stickMotorTorque = m_stickGearbox.getTorque(stickCurrent);

        // Brake engaged when solenoid engaged, disengaged otherwise.
        var boomBrakeEngaged = m_boomBrakeSol.getOutput(); 

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

        // Boom can only rotate if brake is released
        if(boomBrakeEngaged){
            // Presume no slip, static friciton. Force is whatever it needs to be to counter out
            // all other incoming forces, making accel & velocity zero.
            boomAlpha_radpersec2 = 0;
            curBoomAngSpd_radpersec = 0;
        }  else {
            //Normal boom integration
            curBoomAngSpd_radpersec += boomAlpha_radpersec2 * Constants.SIM_SAMPLE_RATE_SEC;
            curBoomAngle_rad += curBoomAngSpd_radpersec * Constants.SIM_SAMPLE_RATE_SEC;
        }

        // Stick can always freely rotate
        curStickAngSpd_radpersec += stickAlpha_radpersec2 * Constants.SIM_SAMPLE_RATE_SEC;
        curStickAngle_rad += curStickAngSpd_radpersec * Constants.SIM_SAMPLE_RATE_SEC;


        // Finally, we set our simulated encoder's readings and simulated battery voltage
        boomAbsEnc.setRawAngle(Constants.ARM_BOOM_ENCODER_MOUNT_OFFSET_RAD + curBoomAngle_rad);
        stickAbsEnc.setRawAngle((Constants.ARM_STICK_ENCODER_MOUNT_OFFSET_RAD + curStickAngle_rad)*-1.0);

        // Update the telemetry for the actual arm position
        var boomAngleDeg = Units.radiansToDegrees(curBoomAngle_rad);
        var stickAngleDeg = Units.radiansToDegrees(curStickAngle_rad);
        ArmTelemetry.getInstance().setActual(boomAngleDeg, stickAngleDeg);
    }
        
}
