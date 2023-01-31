package frc.lib.Util;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 *Draws a diamond shaped marker on a Mechanism 2d
 */
public class Mechanism2DMarker {

    private final Mechanism2DPolygon poly;

    public Mechanism2DMarker(Mechanism2d mechToDrawOn, String name, double size){
        ArrayList<Translation2d> polyPoints = new ArrayList<Translation2d>(Arrays.asList(
            new Translation2d(size,0),
            new Translation2d(0,size),
            new Translation2d(-size,0),
            new Translation2d(0,-size),
            new Translation2d(size,0)
        ));
    
        poly = new Mechanism2DPolygon(mechToDrawOn, name, polyPoints);
    
    }

    public void setLocation(Translation2d location){
        poly.setOrigin(location);
    }

    public void setStyle(Color8Bit color, int lineWidth){
        poly.setStyle(color, lineWidth);
    }

    public void setVisible(boolean isVisible){
        this.poly.setVisible(isVisible);
    }

    
}
