package frc.lib.Faults;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    final String faultActiveTopicName = "faultActive";
    final String faultDescriptionTopicName = "faultDescription";

    private FaultWrangler(){
        faultList = Collections.synchronizedList(new ArrayList<Fault>());

	    Thread bgThread = new Thread(new Runnable() {
	        @Override
	        public void run() {
	            try {
	            	while(!Thread.currentThread().isInterrupted()){
	            		update();
	            		Thread.sleep(1500);
	            	}
	            } catch (Exception e) {
	                e.printStackTrace();
	            }
	        }
		});

        //Set up thread properties and start it off
	    bgThread.setName("FaultWrangler");
	    bgThread.setPriority(Thread.MIN_PRIORITY);
	    bgThread.start();

        
    }

    private void update(){
    
        var activeFaultList = new ArrayList<Fault>();
        for(var fault : faultList){
            if(fault.isActive){
                activeFaultList.add(fault);
            }
        }

        var numActiveFaults = activeFaultList.size();

        if(numActiveFaults > 0){
            curDisplayedFaultIdx = (curDisplayedFaultIdx + 1) % numActiveFaults;
            curFaultStr = activeFaultList.get(curDisplayedFaultIdx).faultStr;
        } else {
            curFaultStr = "";
        }

        SmartDashboard.putBoolean(faultActiveTopicName, curFaultStr.length() > 0);
        SmartDashboard.putString(faultDescriptionTopicName, curFaultStr);
    }

    public String getFaultActiveTopic(){
        return "/SmartDashboard/"+faultActiveTopicName;
    }
    
    public String getFaultDescriptionTopic(){
        return "/SmartDashboard/"+faultDescriptionTopicName;
    }

    public void register(Fault in){
        faultList.add(in);
    }
}
