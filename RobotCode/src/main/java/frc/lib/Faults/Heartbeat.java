package frc.lib.Faults;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import frc.Constants;

public class Heartbeat {

    DigitalOutput ledOut;
    private final double BLINK_FREQ_HZ = 2.0;
    double ledBrightness = 0;

    public Heartbeat(){
        ledOut = new DigitalOutput(Constants.HEARTBEAT_LED_OUT_IDX);
        ledOut.enablePWM(0.0);

    }

    public void ledUpdate() {
        ledBrightness = Math.sin(2 * Math.PI * Timer.getFPGATimestamp() * BLINK_FREQ_HZ / 2.0);
        ledBrightness = Math.max(0, ledBrightness);
        ledOut.updateDutyCycle(ledBrightness);
    }

    
}
