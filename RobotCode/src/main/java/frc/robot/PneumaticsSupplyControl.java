package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.Constants;
import frc.lib.Faults.Fault;
import frc.lib.Signal.Annotations.Signal;

public class PneumaticsSupplyControl {

    AnalogInput pressureSensor;

    Compressor phCompressor;

    @Signal(units = "PSI")
    double storagePressure;

    @Signal(units = "A")
    double compressorCurrent;

    @Signal
    boolean compressorEnableCmd;
    boolean compressorEnableCmdPrev;

    Fault lowPressure = new Fault("Pneumatics", "Low system pressure");
    Fault badSensor = new Fault("Pneumatics", "Sensor unplugged or broken");

    private static PneumaticsSupplyControl inst = null;

    public static synchronized PneumaticsSupplyControl getInstance() {
        if (inst == null)
            inst = new PneumaticsSupplyControl();
        return inst;
    }

    private PneumaticsSupplyControl() {
        phCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        phCompressor.enableDigital();

        pressureSensor = new AnalogInput(Constants.PRESSURE_SENSOR_ANALOG);

        // Kick off monitor in brand new thread.
        // Thanks to Team 254 for an example of how to do this!
        Thread monitorThread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    while (!Thread.currentThread().isInterrupted()) {
                        update();
                        Thread.sleep(250);
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    e.printStackTrace();                
                }

            }
        });

        // Set up thread properties and start it off
        monitorThread.setName("PneumaticsSupplyControl");
        monitorThread.setPriority(Thread.MIN_PRIORITY);
        monitorThread.setDaemon(true);
        monitorThread.start();
    }

    public void setCompressorEnabledCmd(boolean cmd_in) {
        compressorEnableCmd = cmd_in;
    }

    private void update() {

        double voltage = pressureSensor.getVoltage();
        if (voltage >= 0.001) {
            storagePressure = (120.0/151.0 * (250.0 * (voltage / 4.62) - 25.0));
            badSensor.clearFault();
        } else {
            storagePressure = 0.0;// meh, should never happen physically
            badSensor.reportFault();
        }

        compressorCurrent = phCompressor.getCurrent();

        if (compressorEnableCmdPrev != compressorEnableCmd) {
            if (compressorEnableCmd) {
                phCompressor.enableDigital();
            } else {
                phCompressor.disable();
            }
        }

        compressorEnableCmdPrev = compressorEnableCmd;
    }

    public double getStoragePressure() {
        return storagePressure;
    }
}