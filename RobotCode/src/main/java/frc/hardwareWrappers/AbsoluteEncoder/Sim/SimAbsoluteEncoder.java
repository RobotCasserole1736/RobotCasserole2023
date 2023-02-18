package frc.hardwareWrappers.AbsoluteEncoder.Sim;

import frc.hardwareWrappers.SimDeviceBanks;
import frc.hardwareWrappers.AbsoluteEncoder.AbstractAbsoluteEncoder;

public class SimAbsoluteEncoder extends AbstractAbsoluteEncoder {
 
    double curAngle_rad;

    double STEPS_PER_REV = 4096.0; //Simulate quantization

    public SimAbsoluteEncoder(int port){
        SimDeviceBanks.addDIDevice(this, port);
    }

    public void setRawAngle(double curAngle_rad) {
        this.curAngle_rad = curAngle_rad;
    }

    @Override
    public double getRawAngle_rad() {
        return Math.round(curAngle_rad * STEPS_PER_REV/2/Math.PI) * 2*Math.PI/STEPS_PER_REV;
    }

    @Override
    public boolean isFaulted() {
        // TODO Auto-generated method stub
        return false;
    }
    
}
