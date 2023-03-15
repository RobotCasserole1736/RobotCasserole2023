package frc.robot.Arm;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import frc.Constants;
import frc.robot.ArmTelemetry;

public class ArmSoftLimits {

    private final double SMALL_DIFF = 0.01;
    private final double LARGE_DIFF = Double.MAX_VALUE;

    //clockwise from bottom nearest robot
    Double XPosLimits[] = {0.6, 0.48, 0.20, 0.20, 1.612, 1.612};
    Double YPosLimits[] = {0.05, 0.9217, 1.116, 2.347, 2.347, 0.05};

    boolean isLimited;

    public ArmSoftLimits() {

        //One time, set up telemetry for limits
        var softLimitPoly = new ArrayList<Translation2d>();
        for(var i = 0; i < XPosLimits.length; i++){
            softLimitPoly.add(new Translation2d(XPosLimits[i], YPosLimits[i]));
        }
        softLimitPoly.add(new Translation2d(XPosLimits[0], YPosLimits[0])); //one more to close to the start
        ArmTelemetry.getInstance().setSoftLimits(softLimitPoly);
    }

    public ArmEndEffectorState applyLimit(ArmEndEffectorState in) {

        // This is the horizontal line method. Loop over each side and see if a
        // horizontal line
        // drawn from the input point to the right crosses the fence volume once and
        // only once.
        // If the input point is inside, then just return the input, else return the
        // closest point
        // on the fence to the input point as clipped position.

        //Additionally, after that process is complete, clip the position to the maximum and minimum radii 
        // of the arm.



        int i;

        //Default the clipped state to be the same as the input state
        ArmEndEffectorState clipPos = new ArmEndEffectorState(in);
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
            // Do nothing, clipPos is already copied from in

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
                
                    if (crossDist < nearDist) {
                        // use this one
                        nearDist = crossDist;
                        clipPos.x = crossx;
                        clipPos.y = crossy;    
                        clipPos.xvel = 0;              
                        clipPos.yvel = 0;              
                       
                    } else {
                        // leave nearest in place
                        ;
                    } 

                    if (p0Dist < nearDist) {
                        // use this one
                        nearDist = p0Dist;
                        clipPos.x = p0x;
                        clipPos.y = p0y;
                        clipPos.xvel = 0;              
                        clipPos.yvel = 0;   
                        
                    } else {
                        // leave nearest in place
                        ;
                    } 
                
                } 
            
            }


        var x = clipPos.x;
        var y = clipPos.y - Constants.ARM_BOOM_MOUNT_HIEGHT;
        // Ensure the input point is "reachable" by scaling it back
        // inside the unit circle of the max extension of the arm.
        double maxRadius = Constants.ARM_BOOM_LENGTH + Constants.ARM_STICK_LENGTH * 0.99999;
        double minRadius = Math.abs(Constants.ARM_BOOM_LENGTH - Constants.ARM_STICK_LENGTH) * 1.00001;
        double reqRadius = Math.sqrt(x * x + y * y);

        if (reqRadius == 0.0) {
            // user was silly, give up
            clipPos.x = minRadius;
            clipPos.y = 0.0 + Constants.ARM_BOOM_MOUNT_HIEGHT;
            clipPos.xvel = 0;
            clipPos.yvel = 0;
        } else if (reqRadius >= maxRadius) {
            //Requested point beyond the reach of the arm
            // Project the point back into the reachable area
            var angle = Math.atan2(y, x);
            clipPos.x = maxRadius * Math.cos(angle);
            clipPos.y = maxRadius * Math.sin(angle) + Constants.ARM_BOOM_MOUNT_HIEGHT;
            clipPos.xvel = 0;
            clipPos.yvel = 0;
        } else if (reqRadius <= minRadius) {
            //Requested point inside the "unreachable" circle near
            // the pivot point. 
            // Project the point back out into the reachable area
            var angle = Math.atan2(y, x);
            clipPos.x = minRadius * Math.cos(angle); 
            clipPos.y = minRadius * Math.sin(angle) + Constants.ARM_BOOM_MOUNT_HIEGHT;
            clipPos.xvel = 0;
            clipPos.yvel = 0;
        }

        // if we've clipped the position in any way, we're limited
        isLimited = !in.equals(clipPos);
  
        return clipPos;

    }

    public boolean isLimited() {
        return isLimited;
    }

}
