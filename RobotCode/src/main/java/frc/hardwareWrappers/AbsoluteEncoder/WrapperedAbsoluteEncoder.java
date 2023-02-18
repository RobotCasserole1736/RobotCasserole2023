package frc.hardwareWrappers.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.UnitUtils;
import frc.hardwareWrappers.AbsoluteEncoder.CANCoder.RealCANCoder;
import frc.hardwareWrappers.AbsoluteEncoder.RevThroughBore.RealRevThroughBoreEncoder;
import frc.hardwareWrappers.AbsoluteEncoder.SRXEncoder.RealSRXEncoder;
import frc.hardwareWrappers.AbsoluteEncoder.Sim.SimAbsoluteEncoder;
import frc.hardwareWrappers.AbsoluteEncoder.ThriftyEncoder.RealThriftyEncoder;
import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.Robot;

public class WrapperedAbsoluteEncoder  {

    AbstractAbsoluteEncoder enc;

    public enum AbsoluteEncType {
        SRXEncoder,
        CANCoder,
        RevThroughBore,
        Thrifty
    }

    @Signal(units="rad")
    double curAngleRad;

    Calibration mountingOffsetCal;


    public WrapperedAbsoluteEncoder(AbsoluteEncType type, String prefix, int id, double dfltMountingOffset_rad){
        if(Robot.isReal()){
            switch(type){
                case SRXEncoder:
                    //ID = digital input
                    enc = new RealSRXEncoder(id);
                    break;
                case CANCoder:
                    //ID = CAN ID
                    enc = new RealCANCoder(id);
                    break;
                case Thrifty:
                    //ID = Analog Input
                    enc = new RealThriftyEncoder(id);
                    break;
                case RevThroughBore:
                    //ID = digital input
                    enc = new RealRevThroughBoreEncoder(id);
                    break;
            }
        } else {
            enc = new SimAbsoluteEncoder(id);
        }
        mountingOffsetCal = new Calibration(prefix + "MountingOffset", "rad", dfltMountingOffset_rad);
    }

    public void update(){
        curAngleRad = UnitUtils.wrapAngleRad( enc.getRawAngle_rad() - mountingOffsetCal.get());
    }

    public double getAngle_rad(){
        return curAngleRad;
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(this.getAngle_rad());
    }

    public boolean isFaulted(){
        return enc.isFaulted();
    }
    
}
