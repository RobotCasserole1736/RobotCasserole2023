package frc.sim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.Constants;
import frc.hardwareWrappers.SimDeviceBanks;
import frc.hardwareWrappers.AbsoluteEncoder.Sim.SimAbsoluteEncoder;
import frc.hardwareWrappers.MotorCtrl.Sim.SimSmartMotor;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.ArmTelemetry;

public class ArmSim {


    private final SingleJointedArmSim m_boom_sim;
    private final SingleJointedArmSim m_stick_sim;

    private final double ARM_STICK_MASS = Units.lbsToKilograms(5);
    private final double ARM_BOOM_MASS = Units.lbsToKilograms(5);

    SimSmartMotor boomMotorCtrl;
    SimSmartMotor stickMotorCtrl;
    SimAbsoluteEncoder boomAbsEnc;
    SimAbsoluteEncoder stickAbsEnc;

    // driven by one neo
    private final DCMotor m_boomGearbox = DCMotor.getNEO(1);
    private final DCMotor m_stickGearbox = DCMotor.getNEO(1);

    public ArmSim(){
        boomMotorCtrl = (SimSmartMotor) SimDeviceBanks.getCANDevice(Constants.ARM_BOOM_MOTOR_CANID);
        stickMotorCtrl = (SimSmartMotor) SimDeviceBanks.getCANDevice(Constants.ARM_STICK_MOTOR_CANID);

        boomAbsEnc = (SimAbsoluteEncoder) SimDeviceBanks.getDIDevice(Constants.ARM_BOOM_ENC_IDX);
        stickAbsEnc = (SimAbsoluteEncoder) SimDeviceBanks.getDIDevice(Constants.ARM_STICK_ENC_IDX);


        m_boom_sim = new SingleJointedArmSim(
            m_boomGearbox,
            Constants.ARM_BOOM_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(Constants.ARM_BOOM_LENGTH, ARM_BOOM_MASS),
            Constants.ARM_BOOM_LENGTH,
            Units.degreesToRadians(-180),
            Units.degreesToRadians(180),
            ARM_BOOM_MASS,
            true,
            VecBuilder.fill(0) // no noise
            );

        m_stick_sim = new SingleJointedArmSim(
                m_stickGearbox,
                Constants.ARM_STICK_GEAR_RATIO,
                SingleJointedArmSim.estimateMOI(Constants.ARM_STICK_LENGTH, ARM_STICK_MASS),
                Constants.ARM_STICK_LENGTH,
                Units.degreesToRadians(-180),
                Units.degreesToRadians(180),
                ARM_STICK_MASS,
                true,
                VecBuilder.fill(0) //no noise
                );
    }

    public void update(boolean isDisabled){
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        m_boom_sim.setInput(isDisabled ? 0.0 : boomMotorCtrl.getAppliedVoltage_V());
        m_stick_sim.setInput(isDisabled ? 0.0 : stickMotorCtrl.getAppliedVoltage_V());

        // Next, we update it. The standard loop time is 20ms.
        m_boom_sim.update(Constants.SIM_SAMPLE_RATE_SEC);
        m_stick_sim.update(Constants.SIM_SAMPLE_RATE_SEC);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        boomAbsEnc.setRawAngle(m_boom_sim.getAngleRads());
        stickAbsEnc.setRawAngle(m_stick_sim.getAngleRads());

        // Update the Mechanism Arm angle based on the simulated arm angle
        var boomAngle = Units.radiansToDegrees(m_boom_sim.getAngleRads());
        var stickAngle = Units.radiansToDegrees(m_stick_sim.getAngleRads());
        ArmTelemetry.getInstance().setActual(boomAngle, stickAngle);
    }
        
}
