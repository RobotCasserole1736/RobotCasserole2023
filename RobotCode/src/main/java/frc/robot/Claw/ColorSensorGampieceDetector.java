package frc.robot.Claw;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.Faults.Fault;

public class ColorSensorGampieceDetector {

    public boolean clawHasCube;
    public boolean clawHasCone;

    
    private final I2C.Port i2cPort = I2C.Port.kMXP;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    Fault disconFault = new Fault("Claw Color Sensor", "Disconnected");


    public ColorSensorGampieceDetector(){

    }

    public void update(){

        Color detectedColor = m_colorSensor.getColor();

        disconFault.set(!m_colorSensor.isConnected());

        int proximityForGamePiece = m_colorSensor.getProximity();

        boolean doWeHaveGamepiece = false;
        String coneOrCubeOrNothing = "Nothing";
        boolean hasCube = false;
        boolean hasCone = false;

        if (proximityForGamePiece >= 135) {
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