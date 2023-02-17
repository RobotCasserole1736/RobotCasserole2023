package frc.robot.Arm;

import java.util.ArrayList;
import java.util.List;

public class ArmSoftLimits {

    private final double SMALL_DIFF = 0.01;
    private final double LARGE_DIFF = Double.MAX_VALUE;

    public ArmSoftLimits() {

        // Init a new set of bounding box coordinates




    }

    public ArmEndEffectorState applyLimit(ArmEndEffectorState in) {

        // This is the horizontal line method. Loop over each side and see if a
        // horizontal line
        // drawn from the input point to the right crosses the fence volume once and
        // only once.
        // If the input point is inside, then just return the input, else return the
        // closest point
        // on the fence to the input point as clipped position.



        Double XPosLimits[] = {0.0,10.0,10.0,0.0};
        Double YPosLimits[] = {0.0,0.0,10.0,10.0};
        int i;

        ArmEndEffectorState clipPos = new ArmEndEffectorState();
        double diffx = 0.0;
        double diffy = 0.0;
        double p0x;
        double p0y;
        double p1x;
        double p1y;
        double crossx;
        double crossy;

        double m;
        double nearDist, p0Dist, crossDist;
        int crossCount = 0;
    
        for ( i = 0; i < YPosLimits.length; i++) {

            // set up p0 and p1 for this segment
            p0x = XPosLimits[i];
            p0y = YPosLimits[i];
            if (i + 1 == YPosLimits.length) {
                p1x = XPosLimits[0]; // close the boundary back onto the first point
                p1y = YPosLimits[0];
            } else {
                p1x = XPosLimits[i + 1];
                p1y = YPosLimits[i + 1];
            }

            diffx = p1x - p0x;
            diffy = p1y - p0y; 

            if (Math.abs(diffy) < SMALL_DIFF) {
                // horizontal line test (horizontal line does not cross a horizontal line)
                ;
            } else if (Math.abs(diffx) < SMALL_DIFF) {
                // vertical line
                if (in.x < Math.min(p0x, p1x) && in.y > Math.min(p0y, p1y) && in.y <= Math.max(p0y, p1y)) {
                    // crosses this vertical line
                    crossCount++;
                } else {
                    // does not cross the vertical line
                    
                }
            } else if (in.y > Math.min(p0y, p1y) && in.y <= Math.max(p0y, p1y)) {
                // boundary not horiz and not vert and in.y is in range of this segment
                crossx = (in.y - p0y) * diffx / diffy + p0x;
                if (in.x < crossx) {
                    // horizontal line crosses the boundary
                    crossCount++;
                } else {
                    // do nothing out of range
                    ;
                }
            }
        }

        if (crossCount == 1) {
            // inside so no need to clip
            clipPos.x = in.x;
            clipPos.y = in.y;
        } else {
            // it is outside of the fence so clip to nearest point on the boundary
            nearDist = LARGE_DIFF;

            for (i = 0; i < YPosLimits.length; i++) {

                // set up p0 and p1 for this segment
                p0x = XPosLimits[i];
                p0y = YPosLimits[i];

                if (i + 1 == YPosLimits.length) {
                    p1x = XPosLimits[0]; // close the boundary back onto the first point
                    p1y = YPosLimits[0];
                } else {
                    p1x = XPosLimits[i + 1];
                    p1y = YPosLimits[i + 1];
                }
                diffx = p1x - p0x;
                diffy = p1y - p0y;

                // calc p0 to in distance
                p0Dist = Math.sqrt(Math.pow(in.x - p0x, 2) + Math.pow(in.y - p0y, 2)); // problem area starts around here maybe
                if (Math.abs(diffy) < SMALL_DIFF) {
                    // horiz
                    crossx = in.x;
                    crossy = p0y;

                    if (crossx > Math.min(p0x, p1x) && crossx <= Math.max(p0x, p1x)) {
                        // in range
                        crossDist = Math.abs(in.y - crossy);
                    } else {
                        // out of range
                        crossDist = LARGE_DIFF;
                    }
               } else if (Math.abs(diffx) < SMALL_DIFF) {
                    // vert
                    crossx = p0x;
                    crossy = in.y;
                    if (crossy > Math.min(p0y, p1y) && crossy <= Math.max(p0y, p1y)) {
                        // in range
                        crossDist = Math.abs(in.x - crossx);
                    } else {
                        // out of range
                        crossDist = LARGE_DIFF;
                    }
                } else {
                    // not horiz or vert
                    m = diffy / diffx; // slope
                    crossx = (m * p0x - p0y + in.x / m + in.y) / (m + 1.0 / m);
                    crossy = m * crossx - m * p0x + p0y;
                    if ((crossx > Math.min(p0x, p1x) && crossx <= Math.max(p0x, p1x)) &&
                        (crossy > Math.min(p0y, p1y) && crossy <= Math.max(p0y, p1y))) { 
                        // in range
                        crossDist = Math.sqrt(Math.pow(in.x - crossx, 2) + Math.pow(in.y - crossy, 2));
                    } else {
                        // out of range
                        crossDist = LARGE_DIFF;
                    }
                }
                
                System.out.println(crossx + " cross " + crossy);

                    if (crossDist < nearDist) {
                        // use this one
                        nearDist = crossDist;
                        clipPos.x = crossx;
                        clipPos.y = crossy;                  
                       
                    } else {
                        // leave nearest in place
                        ;
                    } 

                    if (p0Dist < nearDist) {
                        // use this one
                        nearDist = p0Dist;
                        clipPos.x = p0x;
                        clipPos.y = p0y;

                        System.out.println(clipPos.x + " clip pos malo" + clipPos.y);
                        
                    } else {
                        // leave nearest in place
                        ;
                    } 
                
                } 
            
            }
  
        return clipPos;
  
        
    
    


    }

}
