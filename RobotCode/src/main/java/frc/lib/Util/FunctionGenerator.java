package frc.lib.Util;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.Calibration.Calibration;

public class FunctionGenerator {

    Calibration sigType;
    Calibration frequency;
    Calibration amplititude;
    Calibration offset;

    double startTime = 0;

    double curOut = 0;
    double prevOut = 0;
    double curTime = Timer.getFPGATimestamp();
    double prevTime = curTime;

    // Source for time-varying signals, used to calibrate closed loop control algorithms and systems
    public FunctionGenerator(String name, String units){
        sigType =  new Calibration("fg_" + name + "_type", "", 0, 0, 3);
        frequency =  new Calibration("fg_" + name + "_freq", "Hz", 0.5, 0, 1/0.02);
        amplititude =  new Calibration("fg_" + name + "_amp", units, 5.0);
        offset =  new Calibration("fg_" + name + "_offset", units, 0);
    }

    public void reset(){
        startTime = Timer.getFPGATimestamp();
        prevTime = curTime = 0.0;
        prevOut = curOut;
    }


    public double getValDeriv(){
        return (curOut - prevOut)/(curTime - prevTime);
    }

    public boolean isEnabled(){
        return (int) sigType.get() != 0;
    }

    //Calcualtes the current value and returns it.
    public double getValue(){

        prevOut = curOut;
        prevTime = curTime;

        curTime = Timer.getFPGATimestamp() - startTime;

        double cycleFrac;
        if(frequency.get() != 0.0){
            double period = 1.0/frequency.get();
            cycleFrac = (curTime % period) / period;
        } else {
            cycleFrac = 0.0;
        }

        int curType = (int) sigType.get();
        switch(curType){
            case 0:
            {
                // No output
                curOut = 0;
            }
            break;
            case 1: 
            {
                // Square
                double cycleGain = cycleFrac > 0.5 ? 0.5 : -0.5;
                curOut = offset.get() + amplititude.get() * cycleGain;
            }
            break;
            case 2: 
            {
                // Sawtooth
                curOut = offset.get() + amplititude.get() * cycleFrac;
            }
            break;
            case 3: 
            {
                // Sine
                curOut = offset.get() + amplititude.get() * Math.sin(2*Math.PI*cycleFrac);
            }
            break;
            case 4: 
            {
                // Constant
                curOut = offset.get();
            }
            break;

            // All others are OFF
        }

        return curOut;

    }
    
}
