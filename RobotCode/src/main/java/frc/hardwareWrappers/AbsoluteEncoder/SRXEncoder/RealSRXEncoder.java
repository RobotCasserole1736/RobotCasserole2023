package frc.hardwareWrappers.AbsoluteEncoder.SRXEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.hardwareWrappers.AbsoluteEncoder.AbstractAbsoluteEncoder;
import frc.lib.Faults.Fault;
import frc.lib.Signal.Annotations.Signal;

public class RealSRXEncoder extends AbstractAbsoluteEncoder {

    DigitalInput m_digitalInput;
    DutyCycle m_dutyCycle;

    @Signal(units="Hz")
    double freq;

    @Signal(units="sec")
    double pulsetime;

    Fault disconFault;

    public RealSRXEncoder(int port){
        m_digitalInput = new DigitalInput(port);
        m_dutyCycle = new DutyCycle(m_digitalInput);
        disconFault = new Fault("Encoder " + Integer.toString(port), "Disconnected");
    }

    @Override
    public double getRawAngle_rad() {
        freq = m_dutyCycle.getFrequency(); //Track this for fault mode detection
        var faulted = (freq < 10);
        disconFault.set(faulted);
        if(faulted){
            pulsetime = -1;
            return 0.0;
        } else {        
            pulsetime = m_dutyCycle.getOutput() * (1.0 / freq);
            double anglerad = ((pulsetime - 1E-6) / (4.096E-3 - 1E-6)) * 2 * Math.PI;
            return anglerad;
        }

    }

    @Override
    public boolean isFaulted() {
        // TODO Auto-generated method stub
        return false;
    }

    
}
