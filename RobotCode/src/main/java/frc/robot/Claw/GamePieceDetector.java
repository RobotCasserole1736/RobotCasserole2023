package frc.robot.Claw;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
    Calibration cubeAbsentThresh;
    Calibration coneAbsentThresh;
    Calibration somethingInIntake;
    Debouncer somethingInIntakDebouncer;

    // Boolean to track game piece presence
    @Signal
    boolean clawHasGamePiece;

    @Signal
    boolean clawHadGamePiece;

    @Signal
    boolean somethingIsPresent;

    public boolean ledShouldBlink;
    Debouncer blinkDebouncer = new Debouncer(2.0, DebounceType.kFalling);

    public GamePieceDetector(){
        // Instantiating the Time of Flight Sensor  
        gamepieceDistSensor = new TimeOfFlight(Constants.GAMEPIECE_DIST_SENSOR_CANID);
        gamepieceDistSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        gamepieceDistSensor.setRangeOfInterest(6, 6, 10, 10);
        disconTOFFault = new Fault("Claw TOF Sensor", "Disconnected");

        // Get instance of Game Piece Mode Manager to know which piece is being picked up
        gpmm = GamepieceModeManager.getInstance();
        
        // Thresholds for Cubes and Cones
        cubePresentThresh = new Calibration("Claw Cube Present Threshold", "in", 7);
        cubeAbsentThresh = new Calibration("Claw Cube Absent Threshold", "in", 11);
        conePresentThresh = new Calibration("Claw Cone Present Threshold", "in", 4);
        coneAbsentThresh = new Calibration("Claw Cone Absent Threshold", "in", 8);

        somethingInIntake = new Calibration("Something in intake Threshold", "in", 12);
        somethingInIntakDebouncer = new Debouncer(0.5, DebounceType.kFalling);


        // Initialize with no game piece
        clawHasGamePiece = false;
        clawHadGamePiece = false;
    }

    // Return true if either game piece is detected
    public boolean hasGamepiece() {
        return clawHasGamePiece;
    }
    
    public void update(){
        // Update TOF sensor reading
        gamepieceDistSensorMeas = Units.metersToInches(gamepieceDistSensor.getRange()/1000.0);
        disconTOFFault.set(gamepieceDistSensor.getFirmwareVersion() == 0);    

        clawHadGamePiece = clawHasGamePiece;

        // Determine if game piece is in claw
        // Use two threshold to achieve some hysterisis
        if (gpmm.isConeMode()) {
            if(gamepieceDistSensorMeas < conePresentThresh.get()){
                clawHasGamePiece = true;
            } else if (gamepieceDistSensorMeas > coneAbsentThresh.get()){
                clawHasGamePiece = false;
            } else {
                // maintain gamepiece state
            }
        } else if (gpmm.isCubeMode()) {
            if(gamepieceDistSensorMeas < cubePresentThresh.get()){
                clawHasGamePiece = true;
            } else if (gamepieceDistSensorMeas > cubeAbsentThresh.get()){
                clawHasGamePiece = false;
            } else {
                // maintain gamepiece state
            }
        }
        else {
            clawHasGamePiece = false;
        }

        somethingIsPresent = somethingInIntakDebouncer.calculate( gamepieceDistSensorMeas < somethingInIntake.get());

        if(clawHasGamePiece && !clawHadGamePiece) {
            ledShouldBlink = blinkDebouncer.calculate(true);
        } else {
            ledShouldBlink = blinkDebouncer.calculate(false);
        }
        GamepieceModeManager.getInstance().setBlinkMode(ledShouldBlink);
    } 

    public boolean newGamePiece(){
        return (clawHasGamePiece && !clawHadGamePiece);
    }

    public boolean ledShouldBlink() {
        return ledShouldBlink;
    }

    public boolean getSomethingIsPresent(){
        return somethingIsPresent;
    }
    
}