package frc.hardwareWrappers.MotorCtrl;

import frc.hardwareWrappers.MotorCtrl.Sim.SimSmartMotor;
import frc.hardwareWrappers.MotorCtrl.SparkMax.RealSparkMax;
import frc.hardwareWrappers.MotorCtrl.TalonFX.RealTalonFX;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.Robot;

public class WrapperedCANMotorCtrl {

    public enum CANMotorCtrlType {
        TALON_FX,
        SPARK_MAX
    }

    AbstractSimmableMotorController ctrl;

    @Signal(units="V")
    private double appliedVoltage;


    @Signal(units = "A")
    private double current;

    @Signal(units = "radpersec")
    private double desVel;


    public WrapperedCANMotorCtrl(String prefix, int can_id, CANMotorCtrlType type){

        System.out.print("=> Starting motor controller init for " + prefix + " CANID = " + Integer.toString(can_id));

        if(Robot.isSimulation()){
            ctrl = new SimSmartMotor(can_id);
        } else {
            switch(type){
                case TALON_FX:
                    ctrl = new RealTalonFX(can_id);
                    break;
                case SPARK_MAX:
                    ctrl = new RealSparkMax(can_id);
                    break;
            }
        }
        System.out.println(" ... Done!");

    }
    

    public void update(){
        current = ctrl.getCurrent_A();
        appliedVoltage = ctrl.getAppliedVoltage_V();
    }

    public void setInverted(boolean invert){
        ctrl.setInverted(invert);
    }

    public void setClosedLoopGains(double p, double i, double d){
        ctrl.setClosedLoopGains(p, i, d);
    }

    public void setClosedLoopCmd(double velocityCmd_radpersec, double arbFF_V){
        ctrl.setClosedLoopCmd(velocityCmd_radpersec, arbFF_V);
        desVel = velocityCmd_radpersec;
    }

    public void setVoltageCmd(double cmd_V){
        ctrl.setVoltageCmd(cmd_V);
    }

    public double getCurrent_A(){
        return current;
    }

    public double getVelocity_radpersec(){
        return ctrl.getVelocity_radpersec();
    }

    public double getPosition_rad(){
        return ctrl.getPosition_rad();
    }

    public void resetDistance(){
        ctrl.resetDistance();
        update();
    }

    public void setBrakeMode(boolean brakeMode){
        ctrl.setBrakeMode(brakeMode);
    }


}
