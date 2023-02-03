package frc.hardwareWrappers.MotorCtrl.SparkMax;


import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.UnitUtils;
import frc.hardwareWrappers.MotorCtrl.AbstractSimmableMotorController;
import frc.lib.Faults.Fault;


public class RealSparkMax extends AbstractSimmableMotorController {

    private CANSparkMax m_motor;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;

    private final int MAX_RETRIES = 10;
    boolean connected = false;

    Fault disconnFault;

    public RealSparkMax(int can_id){
        m_motor = new CANSparkMax(can_id, MotorType.kBrushless);

        boolean success = false;
        int retryCount = 0;

        disconnFault = new Fault("Spark Max ID " + Integer.toString(can_id), "Disconnected");

        while(!success && retryCount < MAX_RETRIES){    
            var err0 = m_motor.restoreFactoryDefaults();
            var err1 = m_motor.setIdleMode(IdleMode.kCoast);
            var err2 = m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 19);// Status 0 = Motor output and Faults
            var err3 = m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 57);// Status 1 = Motor velocity & electrical data
            var err4 = m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65500);// Status 2 = Motor Position
            var err5 = m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65500);// Status 3 = Analog Sensor Input
            var err6 = m_motor.setSmartCurrentLimit(40); // 40 A current limit
            success = (err0 == REVLibError.kOk &&
                       err1 == REVLibError.kOk &&
                       err2 == REVLibError.kOk &&
                       err3 == REVLibError.kOk &&
                       err4 == REVLibError.kOk &&
                       err5 == REVLibError.kOk &&
                       err6 == REVLibError.kOk );
        
            if(!success){
                System.out.println("Configuration Failed for CAN ID " + Integer.toString(can_id) + ", retrying....");
                retryCount++;
            }
        }

        //we're only connected if we were successful.
        connected = success;

        if(!connected) {
            DriverStation.reportError("Failed to configure motor CAN ID " + Integer.toString(can_id), false);
        } else {
            m_pidController = m_motor.getPIDController();
            m_encoder = m_motor.getEncoder();
        }

        disconnFault.set(!connected);
        
    }


    @Override
    public void setInverted(boolean invert) {
        if(connected){
            m_motor.setInverted(invert);
        }
    }


    @Override
    public void setClosedLoopGains(double p, double i, double d) {

        // I don't know why we need to do this, but it makes sim line up with real life.
        p /= 1000;

        //Convert to Rev units of RPM
        p = Units.radiansPerSecondToRotationsPerMinute(p);
        i = Units.radiansPerSecondToRotationsPerMinute(i);
        d = Units.radiansPerSecondToRotationsPerMinute(d);

        if(connected){
            m_pidController.setP(p);
            m_pidController.setI(i);
            m_pidController.setD(d);
            m_pidController.setOutputRange(-1.0, 1.0);
        }
    }


    @Override
    public void setClosedLoopCmd(double velocityCmd_radpersec, double arbFF_V) {
        if(connected){

            m_pidController.setReference(Units.radiansPerSecondToRotationsPerMinute(velocityCmd_radpersec), 
                                        CANSparkMax.ControlType.kVelocity,
                                        0,
                                        arbFF_V,
                                        SparkMaxPIDController.ArbFFUnits.kVoltage);
        }
    }


    @Override
    public void setVoltageCmd(double cmd_v) {
        if(connected){
            m_motor.setVoltage(cmd_v);
        }
    }


    @Override
    public double getCurrent_A() {
        if(connected){
            return m_motor.getOutputCurrent();
        } else {
            return 0;
        }
    }


    @Override
    public double getVelocity_radpersec() {
        if(connected){
            return  Units.degreesToRadians(UnitUtils.RPMtoDegPerSec(m_encoder.getVelocity()));
        } else {
            return 0;
        }
    }


    @Override
    public void follow(Object leader) {
        if(leader.getClass() == RealSparkMax.class){
            this.m_motor.follow(((RealSparkMax)leader).m_motor);
        } else {
            throw new IllegalArgumentException(leader.getClass().toString() + " cannot be followed by a " + this.getClass().toString());
        }

    }

    @Override
    public double getPosition_rad() {
        if(connected){
            return Units.rotationsToRadians(m_encoder.getPosition());
        } else {
            return 0;
        }
    }


    @Override
    public double getAppliedVoltage_V() {
        if(connected){
            return m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        } else {
            return 0;
        }
    }

    @Override
    public void resetDistance() {
        if(connected){
            m_encoder.setPosition(0.0);
        }
    }


    @Override
    public void setBrakeMode(boolean isBrakeMode) {
        boolean success = false;
        int retryCount = 0;
        while(!success && retryCount++ < MAX_RETRIES){    
            var err1 = m_motor.setIdleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
            success = err1 == REVLibError.kOk;
        }

        if(!success) {
            DriverStation.reportError("Failed to configure coast/brake mode for motor CAN ID " + Integer.toString(this.m_motor.getDeviceId()), false);
        }

        
    }


}
