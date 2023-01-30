package frc.lib.Util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Square peg into round hole.
 * 
 * Abuse the mechanism 2d ligament/root symantecs to 
 * display arbitrary polygons
 */
public class Mechanism2DPolygon {

    private final Mechanism2d m_mech2d; 
    private final String name;
    private List<Translation2d> polygon;
    private Translation2d origin;
    private Color8Bit color;
    private int lineWidth;
    private boolean isVisible;

    private MechanismRoot2d mechRoot;
    private List<MechanismLigament2d> mechLigaments;

    public Mechanism2DPolygon(Mechanism2d mechToDrawOn, String name, List<Translation2d> polygon){
        this.m_mech2d = mechToDrawOn;
        this.name = name;
        this.origin = new Translation2d(); //default about mech2d origin
        this.color = new Color8Bit(Color.kSilver);
        this.lineWidth = 2;
        this.isVisible = true;
        this.mechRoot = m_mech2d.getRoot(rootName(), 0,0);
        this.mechLigaments = new ArrayList<MechanismLigament2d>(polygon.size());
        setPolygon(polygon);
    }

    public void setPolygon(List<Translation2d> polygon){
        this.polygon = polygon;
        draw();
    }

    public void clear(){
        this.polygon.clear();
        draw();
    }

    public void addPoint(Translation2d point){
        this.polygon.add(point);
    }

    public void setOrigin(Translation2d origin){
        this.origin = origin;
        draw();
    }

    public void setStyle(Color8Bit color, int lineWidth){
        this.color = color;
        this.lineWidth = lineWidth;
        draw();
    }

    public void setVisible(boolean isVisible){
        this.isVisible = isVisible;
        draw();
    }

    private void draw(){

        //Hide all ligaments
        for(MechanismLigament2d lig : this.mechLigaments){
            lig.setLength(0);
            lig.setLineWeight(0);
        }

        Rotation2d prevAngle = new Rotation2d();

        if(this.isVisible){
            for(int segIdx = 0; segIdx < this.polygon.size()-1; segIdx++){
                var fromPoint = this.polygon.get(segIdx).plus(origin);
                var toPoint = this.polygon.get(segIdx+1).plus(origin);

                //Edit the existing ligament, or add a new one if needed.
                // No deletion, we just leave those at zero length and size.
                MechanismLigament2d lig;
                if(segIdx < this.mechLigaments.size()){
                    //Use existing ligament
                    lig = this.mechLigaments.get(segIdx);
                } else {
                    //Make a new ligament and attach it.
                    lig = new MechanismLigament2d(ligName(segIdx), 0, 0);
                    if(segIdx == 0){
                        this.mechRoot.append(lig);
                    } else {
                        this.mechLigaments.get(this.mechLigaments.size()-1).append(lig);
                    }
                    this.mechLigaments.add(lig);

                }

                lig.setColor(this.color);
                lig.setLineWeight(this.lineWidth);

                var len = fromPoint.getDistance(toPoint);
                lig.setLength(len);

                var deltaX = toPoint.getX() - fromPoint.getX();
                var deltaY = toPoint.getY() - fromPoint.getY();
                var angle = new Rotation2d(deltaX, deltaY);
                lig.setAngle(angle.minus(prevAngle));
                prevAngle = angle;

                if(segIdx == 0){
                    mechRoot.setPosition(fromPoint.getX(), fromPoint.getY());
                }

            }

        }

    }

    private String rootName(){
        return this.name + "_PolyRoot";
    }

    private String ligName(int idx){
        return this.name + "_PolyEdge" + Integer.toString(idx);
    }
    
}
