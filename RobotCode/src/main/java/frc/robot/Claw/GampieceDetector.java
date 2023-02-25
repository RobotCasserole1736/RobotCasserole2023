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

public class GampieceDectector {
    TimeOfFlight gamepieceDistSensor;
    double gamepieceDistSensorMeas;
    Calibration cubePresentThresh;
    Calibration conePresentThresh;
    public void GampieceDetector(){
            gamepieceDistSensor = new TimeOfFlight(Constants.GAMEPIECE_DIST_SENSOR_CANID);
            gamepieceDistSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
            gamepieceDistSensor.setRangeOfInterest(6, 6, 10, 10);
    
            cubePresentThresh = new Calibration("Claw Cube Present Threshold", "in", 2);
            conePresentThresh = new Calibration("Claw Cone Present Threshold", "in", 3);
        }
    public class ColorSensorGampieceDetector {

        public boolean clawHasCube;
        public boolean clawHasCone;
        //booleans that determine object type
        
        private final I2C.Port i2cPort = I2C.Port.kMXP;
        private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
        //Sensors used to identify objects
        Fault disconFault = new Fault("Claw Color Sensor", "Disconnected");
    
    
        public ColorSensorGampieceDetector(){
    
        }
    
        public void update(){
    
            Color detectedColor = m_colorSensor.getColor();
    
            disconFault.set(!m_colorSensor.isConnected());
    
            
    
            boolean doWeHaveGamepiece = false;
            //unused?
            String coneOrCubeOrNothing = "Nothing";
            boolean hasCube = false;
            boolean hasCone = false;
    
            // Proximity sensor: Farther away = smaller, closer = larger 
            if (gamepieceDistSensorMeas < conePresentThresh.get()) {
                doWeHaveGamepiece = true;
                if (Math.abs(detectedColor.red - detectedColor.blue) > Math.abs(detectedColor.green - detectedColor.blue)  ) {
                    coneOrCubeOrNothing = "Cube";
                    hasCube = true;
                    hasCone = false;
                   
                }else {
                    coneOrCubeOrNothing = "Cone";
                    hasCone = true;
                    hasCube = false;
                }
    
            } else {
                doWeHaveGamepiece = false;
            }
            
            clawHasCone = hasCone;
            clawHasCube = hasCube;
        }
    
        public boolean hasGamepiece() {
            boolean gamepiecePresent;
            if (clawHasCone == true || clawHasCube == true) {
                gamepiecePresent = true;
            } else {
                gamepiecePresent = false;
            }
            return gamepiecePresent; 
        }
    
    
    }
    
    public class TOFGampieceDetector {

        
    
        
        @Signal(units="in")
        double gamepieceDistSensorMeas = 0.0;
    
        boolean hasGamepiece = false;
    
        Calibration cubePresentThresh;
        Calibration conePresentThresh;
    
        Fault disconFault = new Fault("Claw TOF Sensor", "Disconnected");
    
    
        
    
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
}
