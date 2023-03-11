package frc.robot.Claw;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PWM;
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

    private final double PWM_HASPIECE = -1.0;
    private final double PWM_NOPIECE = 0.0;
    PWM LEDStripModeCtrl;

    GamepieceModeManager gpmm;
    // Thresholds for Cubes and Cones
    Calibration cubePresentThresh;
    Calibration conePresentThresh;
    Calibration cubeAbsentThresh;
    Calibration coneAbsentThresh;

    // Boolean to track game piece presence
    boolean clawHasGamePiece;
    boolean clawHadGamePiece;

    public boolean ledShouldBlink;
    Debouncer blinkDebouncer = new Debouncer(1.0, DebounceType.kFalling);

    public GamePieceDetector(){
        // Instantiating the Time of Flight Sensor  
        gamepieceDistSensor = new TimeOfFlight(Constants.GAMEPIECE_DIST_SENSOR_CANID);
        gamepieceDistSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        gamepieceDistSensor.setRangeOfInterest(6, 6, 10, 10);
        disconTOFFault = new Fault("Claw TOF Sensor", "Disconnected");

        // Get instance of Game Piece Mode Manager to know which piece is being picked up
        gpmm = GamepieceModeManager.getInstance();
        
        // Thresholds for Cubes and Cones
        cubePresentThresh = new Calibration("Claw Cube Present Threshold", "in", 5);
        cubeAbsentThresh = new Calibration("Claw Cube Absent Threshold", "in", 9);
        conePresentThresh = new Calibration("Claw Cone Present Threshold", "in", 4);
        coneAbsentThresh = new Calibration("Claw Cone Absent Threshold", "in", 8);

        // Initialize with no game piece
        clawHasGamePiece = false;
        clawHadGamePiece = false;

        // Initialize LED Strip PWM signal
        LEDStripModeCtrl = new PWM(Constants.LED_STRIP_PORT);
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
                LEDStripModeCtrl.setSpeed(PWM_HASPIECE);
            } else if (gamepieceDistSensorMeas > coneAbsentThresh.get()){
                clawHasGamePiece = false;
            } else {
                // maintain gamepiece state
            }
        } else if (gpmm.isCubeMode()) {
            if(gamepieceDistSensorMeas < cubePresentThresh.get()){
                clawHasGamePiece = true;
                LEDStripModeCtrl.setSpeed(PWM_HASPIECE);
            } else if (gamepieceDistSensorMeas > cubeAbsentThresh.get()){
                clawHasGamePiece = false;
            } else {
                // maintain gamepiece state
            }
        }
        else {
            clawHasGamePiece = false;
            LEDStripModeCtrl.setSpeed(PWM_NOPIECE);
        }

        if(clawHasGamePiece && !clawHadGamePiece) {
            ledShouldBlink = blinkDebouncer.calculate(true);
        } else {
            ledShouldBlink = blinkDebouncer.calculate(false);
        }

    } 

    public boolean newGamePiece(){
        return (clawHasGamePiece && !clawHadGamePiece);
    }

    public boolean ledShouldBlink() {
        return ledShouldBlink;
    }
    
}