package frc.hardwareWrappers.AbsoluteEncoder.ThriftyEncoder;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.hardwareWrappers.AbsoluteEncoder.AbstractAbsoluteEncoder;
import frc.lib.Signal.Annotations.Signal;

public class RealThriftyEncoder extends AbstractAbsoluteEncoder {

    AnalogInput m_input;
    AnalogEncoder m_encoder;

    @Signal(units="V")
    double measVoltage;

    public RealThriftyEncoder(int port){
        m_input = new AnalogInput(port);
        m_encoder = new AnalogEncoder(m_input);
    }

    @Override
    public double getRawAngle_rad() {
        measVoltage = m_input.getVoltage();
        return m_encoder.getAbsolutePosition();
    }

    
}
