package frc.sim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.Constants;
import frc.hardwareWrappers.SimDeviceBanks;
import frc.hardwareWrappers.AbsoluteEncoder.Sim.SimAbsoluteEncoder;
import frc.hardwareWrappers.MotorCtrl.Sim.SimSmartMotor;
import frc.robot.ArmTelemetry;

public class ArmSim {


    private final SingleJointedArmSim m_arm_upper_sim;
    private final SingleJointedArmSim m_arm_lower_sim;

    
    private final int m_arm_upper_min_angle = -180; 
    private final int m_arm_upper_max_angle = 260; 
    private final int m_arm_lower_min_angle = -90; 
    private final int m_arm_lower_max_angle = 190; 


    SimSmartMotor upperMotorCtrl;
    SimSmartMotor lowerMotorCtrl;
    SimAbsoluteEncoder upperAbsEnc;
    SimAbsoluteEncoder lowerAbsEnc;

    // driven by one neo
    private final DCMotor m_armUpperGearbox = DCMotor.getNEO(1);
    private final DCMotor m_armLowerGearbox = DCMotor.getNEO(1);

    public ArmSim(){
        upperMotorCtrl = (SimSmartMotor) SimDeviceBanks.getCANDevice(Constants.ARM_UPPER_MOTOR_CANID);
        lowerMotorCtrl = (SimSmartMotor) SimDeviceBanks.getCANDevice(Constants.ARM_LOWER_MOTOR_CANID);

        upperAbsEnc = (SimAbsoluteEncoder) SimDeviceBanks.getDIDevice(Constants.ARM_UPPER_ENC_IDX);
        lowerAbsEnc = (SimAbsoluteEncoder) SimDeviceBanks.getDIDevice(Constants.ARM_LOWER_ENC_IDX);


        m_arm_upper_sim = new SingleJointedArmSim(
            m_armUpperGearbox,
            Constants.ARM_UPPER_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(Constants.ARM_UPPER_LENGTH, Constants.ARM_UPPER_MASS),
            Constants.ARM_UPPER_LENGTH,
            Units.degreesToRadians(m_arm_upper_min_angle),
            Units.degreesToRadians(m_arm_upper_max_angle),
            Constants.ARM_UPPER_MASS,
            false,
            VecBuilder.fill(0) // no noise
            );

        m_arm_lower_sim = new SingleJointedArmSim(
                m_armLowerGearbox,
                Constants.ARM_LOWER_GEAR_RATIO,
                SingleJointedArmSim.estimateMOI(Constants.ARM_LOWER_LENGTH, Constants.ARM_UPPER_MASS),
                Constants.ARM_LOWER_LENGTH,
                Units.degreesToRadians(m_arm_lower_min_angle),
                Units.degreesToRadians(m_arm_lower_max_angle),
                Constants.ARM_LOWER_MASS,
                true,
                VecBuilder.fill(0) //no noise
                );
    }

    public void update(boolean isDisabled){
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        m_arm_upper_sim.setInput(isDisabled ? 0.0 : upperMotorCtrl.getAppliedVoltage_V());
        m_arm_lower_sim.setInput(isDisabled ? 0.0 : lowerMotorCtrl.getAppliedVoltage_V());

        // Next, we update it. The standard loop time is 20ms.
        m_arm_upper_sim.update(Constants.SIM_SAMPLE_RATE_SEC);
        m_arm_lower_sim.update(Constants.SIM_SAMPLE_RATE_SEC);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        upperAbsEnc.setRawAngle(m_arm_upper_sim.getAngleRads());
        lowerAbsEnc.setRawAngle(m_arm_lower_sim.getAngleRads());

        // Update the Mechanism Arm angle based on the simulated arm angle
        var upperAngle = Units.radiansToDegrees(m_arm_upper_sim.getAngleRads());
        var lowerAngle = Units.radiansToDegrees(m_arm_lower_sim.getAngleRads());
        ArmTelemetry.getInstance().setActual(upperAngle, lowerAngle);
    }
        
}
