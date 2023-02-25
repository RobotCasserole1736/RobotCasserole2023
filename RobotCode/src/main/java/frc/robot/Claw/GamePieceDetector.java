package frc.robot.Claw;

import com.revrobotics.ColorSensorV3;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.util.Units;
import frc.Constants;
import frc.lib.Calibration.Calibration;
import frc.lib.Faults.Fault;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.GamepieceModeManager;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class GamePieceDetector {
    // Time of Flight sensor and double to track distance
    TimeOfFlight gamepieceDistSensor;
    @Signal(units="in") // Not sure if this is necessary
    double gamepieceDistSensorMeas;

    // Color sensor and color variable
    
    Color detectedColor;

    // Thresholds for Cubes and Cones
    Calibration cubePresentThresh;
    Calibration conePresentThresh;
    
    // Booleans that determine object type
    boolean clawHasCube;
    boolean clawHasCone;

    public GamePieceDetector(){
        // Instantiating the Time of Flight Sensor  
        gamepieceDistSensor = new TimeOfFlight(Constants.GAMEPIECE_DIST_SENSOR_CANID);
        gamepieceDistSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        gamepieceDistSensor.setRangeOfInterest(6, 6, 10, 10);

        // Thresholds for Cubes and Cones
        cubePresentThresh = new Calibration("Claw Cube Present Threshold", "in", 2);
        conePresentThresh = new Calibration("Claw Cone Present Threshold", "in", 3);
        
        // Initialize with no game piece
        clawHasCube = false;
        clawHasCone = false;
    }
    public class ColorSensorGampieceDetector {
        private final I2C.Port i2cPort = I2C.Port.kMXP;
        private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
        // Sensors used to identify objects
        Fault disconFault = new Fault("Claw Color Sensor", "Disconnected");

    // Return true if either game piece is detected
    public boolean hasGamepiece() {
        return clawHasCone || clawHasCube;
    }
    
    public void update(){

        Color detectedColor = m_colorSensor.getColor();
        disconFault.set(!m_colorSensor.isConnected());
        
        // Do we really need this?
        String coneOrCubeOrNothing = "Nothing";

        // Proximity sensor: Farther away = smaller, closer = larger 
        if (gamepieceDistSensorMeas < conePresentThresh.get()) {
            if (Math.abs(detectedColor.red - detectedColor.blue) > Math.abs(detectedColor.green - detectedColor.blue)  ) {
                coneOrCubeOrNothing = "Cube";
                clawHasCube = true;
                clawHasCone = false;
                
            } else {
                coneOrCubeOrNothing = "Cone";
                clawHasCube = false;
                clawHasCone = true;
            }
        } else {
            clawHasCone = false;
            clawHasCube = false;
        }
    }
}
    
    public class TOFGampieceDetector {
        
        double gamepieceDistSensorMeas = 0.0;
    
        Calibration cubePresentThresh;
        Calibration conePresentThresh;
    
        Fault disconFault = new Fault("Claw TOF Sensor", "Disconnected");
    
        public void update(){
           
            gamepieceDistSensorMeas = Units.metersToInches(gamepieceDistSensor.getRange()/1000.0); 

            disconFault.set(gamepieceDistSensor.getFirmwareVersion() == 0);    
        }
    }
}
