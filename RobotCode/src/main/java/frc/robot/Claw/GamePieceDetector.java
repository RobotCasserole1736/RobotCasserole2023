package frc.robot.Claw;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.util.Units;
import frc.Constants;
import frc.lib.Calibration.Calibration;
import frc.lib.Faults.Fault;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.GamepieceModeManager;

public class GamePieceDetector {
    // Time of Flight sensor and double to track distance
    TimeOfFlight gamepieceDistSensor;
    Fault disconTOFFault;
    @Signal(units="in") // Not sure if this is necessary
    double gamepieceDistSensorMeas;

    GamepieceModeManager gpmm;
    // Thresholds for Cubes and Cones
    Calibration cubePresentThresh;
    Calibration conePresentThresh;
    
    // Boolean to track game piece presence
    boolean clawHasGamePiece;

    public GamePieceDetector(){
        // Instantiating the Time of Flight Sensor  
        gamepieceDistSensor = new TimeOfFlight(Constants.GAMEPIECE_DIST_SENSOR_CANID);
        gamepieceDistSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        gamepieceDistSensor.setRangeOfInterest(6, 6, 10, 10);
        disconTOFFault = new Fault("Claw TOF Sensor", "Disconnected");

        // Get instance of Game Piece Mode Manager to know which piece is being picked up
        gpmm = GamepieceModeManager.getInstance();
        
        // Thresholds for Cubes and Cones
        cubePresentThresh = new Calibration("Claw Cube Present Threshold", "in", 2);
        conePresentThresh = new Calibration("Claw Cone Present Threshold", "in", 3);
        
        // Initialize with no game piece
        clawHasGamePiece = false;
    }

    // Return true if either game piece is detected
    public boolean hasGamepiece() {
        return clawHasGamePiece;
    }
    
    public void update(){
        // Update TOF sensor reading
        gamepieceDistSensorMeas = Units.metersToInches(gamepieceDistSensor.getRange()/1000.0);
        disconTOFFault.set(gamepieceDistSensor.getFirmwareVersion() == 0);    

        // Determine if game piece is in claw
        if (gpmm.isConeMode()) {
            if (gamepieceDistSensorMeas < conePresentThresh.get()) {
                clawHasGamePiece = true;
            }
        } else if (gpmm.isCubeMode()) {
            if (gamepieceDistSensorMeas < cubePresentThresh.get()) {
                clawHasGamePiece = true;
            }
        }
        else {
            clawHasGamePiece = false;
        }
    }
}