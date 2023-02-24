package frc.robot.Claw;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.util.Units;
import frc.Constants;
import frc.lib.Calibration.Calibration;
import frc.lib.Faults.Fault;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.GamepieceModeManager;

public class TOFGampieceDetector {

    TimeOfFlight gamepieceDistSensor;

    
    @Signal(units="in")
    double gamepieceDistSensorMeas = 0.0;

    boolean hasGamepiece = false;

    Calibration cubePresentThresh;
    Calibration conePresentThresh;

    Fault disconFault = new Fault("Claw TOF Sensor", "Disconnected");


    public TOFGampieceDetector(){
        gamepieceDistSensor = new TimeOfFlight(Constants.GAMEPIECE_DIST_SENSOR_CANID);
        gamepieceDistSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        gamepieceDistSensor.setRangeOfInterest(6, 6, 10, 10);

        cubePresentThresh = new Calibration("Claw Cube Present Threshold", "in", 2);
        conePresentThresh = new Calibration("Claw Cone Present Threshold", "in", 3);

    }

    public void update(){

        gamepieceDistSensorMeas = Units.metersToInches(gamepieceDistSensor.getRange()/1000.0); 

        disconFault.set(gamepieceDistSensor.getFirmwareVersion() == 0);


        if(GamepieceModeManager.getInstance().isConeMode()){
            hasGamepiece = gamepieceDistSensorMeas < conePresentThresh.get();
        } else {
            hasGamepiece = gamepieceDistSensorMeas < cubePresentThresh.get();
        }

    }

    public boolean hasGamepiece() {
        return hasGamepiece;
    }


}