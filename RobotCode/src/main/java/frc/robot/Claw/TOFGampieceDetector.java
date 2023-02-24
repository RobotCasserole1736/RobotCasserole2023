package frc.robot;

public class TOFGamepieceDetector {

    TimeOfFlight gamepieceDistSensor;

    
    @Signal(units="in")
    double gamepieceDistSensorMeas = 0.0;

    boolean hasGamepeice = false;

    Calibration cubePresentThresh;
    Calibration conePresentThresh;

    Fault disconFault = new Fault("Claw TOF Sensor", "Disconnected");


    public TOFGamepieceDetector(){
        gamepieceDistSensor = new TimeOfFlight(Constants.GAMEPIECE_DIST_SENSOR_CANID);
        gamepieceDistSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        gamepieceDistSensor.setRangeOfInterest(6, 6, 10, 10);

        cubePresentThresh = new Calibration("Claw Cube Present Threshold", "in", 2);
        conePresentThresh = new Calibration("Claw Cone Present Threshold", "in", 3);

    }

    public void update(){

        gamepieceDistSensorMeas = Units.millimetersToInches(gamepieceDistSensor.getRange()); 

        disconFault.set(gamepieceDistSensor.isConnected());


        if(GamepieceModeManager.getInstance().isConeMode()){
            hasGamepeice = gamepieceDistSensorMeas < conePresentThresh.get();
        } else {
            hasGamepiece = gamepieceDistSensorMeas < cubePresentThresh.get();
        }

    }

    public boolean hasGamepeice() {
        return hasGamepeice;
    }


}