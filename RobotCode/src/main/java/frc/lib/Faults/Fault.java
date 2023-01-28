package frc.lib.Faults;

public class Fault {

    boolean isActive = false;
    boolean prevIsActive = false;
    String faultStr;

    public Fault(String deviceName, String faultDescription){
        faultStr = "[" + deviceName + "] " + faultDescription;
        FaultWrangler.getInstance().register(this);
    }

    public void set(boolean isActive){
        this.isActive = isActive;
    }

    public void reportFault(){
        isActive = true;
    }

    public void clearFault(){
        isActive = false;
    }
    
}
