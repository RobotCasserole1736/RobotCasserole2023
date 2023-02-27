package frc.lib.Faults;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Constants;

public class FaultWrangler {

    /* Singleton infrastructure */
    private static FaultWrangler instance;

    public static FaultWrangler getInstance() {
        if (instance == null) {
            instance = new FaultWrangler();
        }
        return instance;
    }

    List<Fault> faultList;

    String curFaultStr;

    int curDisplayedFaultIdx;
    int numActiveFaults;

    Heartbeat hb;

    final String faultActiveTopicName = "faultActive";
    final String faultDescriptionTopicName = "faultDescription";

    DigitalOutput ledOut;
    private final double BLINK_FREQ_HZ = 2.0;
    double ledBrightness = 0;

    private FaultWrangler() {
        faultList = Collections.synchronizedList(new ArrayList<Fault>());
        ledOut = new DigitalOutput(Constants.FAULT_LED_OUT_IDX);
        ledOut.setPWMRate(500.0);
        ledOut.enablePWM(0.0);

        hb = new Heartbeat();

        Thread bgThread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    while (!Thread.currentThread().isInterrupted()) {
                        messageUpdate();
                        for (int i = 0; i < 30; i++) {
                            ledUpdate();
                            Thread.sleep(50);
                            hb.ledUpdate();
                        }
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });

        // Set up thread properties and start it off
        bgThread.setName("FaultWrangler");
        bgThread.setPriority(Thread.MIN_PRIORITY);
        bgThread.setDaemon(true);
        bgThread.start();


    }

    private void messageUpdate() {

        var activeFaultList = new ArrayList<Fault>();
        for (var fault : faultList) {
            if (fault.isActive) {
                activeFaultList.add(fault);
            }
        }

        numActiveFaults = activeFaultList.size();

        if (numActiveFaults > 0) {
            curDisplayedFaultIdx = (curDisplayedFaultIdx + 1) % numActiveFaults;
            curFaultStr = activeFaultList.get(curDisplayedFaultIdx).faultStr;
        } else {
            curFaultStr = "";
        }

        SmartDashboard.putBoolean(faultActiveTopicName, curFaultStr.length() > 0);
        SmartDashboard.putString(faultDescriptionTopicName, curFaultStr);
    }

    private void ledUpdate() {

        boolean ledActive = (numActiveFaults > 0);
        if (ledActive) {
            ledBrightness = Math.abs(Math.sin(2 * Math.PI * Timer.getFPGATimestamp() * BLINK_FREQ_HZ / 2.0));
            ledOut.updateDutyCycle(ledBrightness);
        } else {
            ledBrightness = 0;
        }
        ledOut.updateDutyCycle(ledBrightness);

    }

    public String getFaultActiveTopic() {
        return "/SmartDashboard/" + faultActiveTopicName;
    }

    public String getFaultDescriptionTopic() {
        return "/SmartDashboard/" + faultDescriptionTopicName;
    }

    public void register(Fault in) {
        faultList.add(in);
    }
}
