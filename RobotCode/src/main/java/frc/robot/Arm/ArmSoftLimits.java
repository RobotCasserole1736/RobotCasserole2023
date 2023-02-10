package frc.robot.Arm;

import java.util.ArrayList;
import java.util.List;

public class ArmSoftLimits {

    private final double SMALL_DIFF = 0.0001;
    private final double LARGE_DIFF = Double.MAX_VALUE;

    private List<ArmEndEffectorState> ArmLim = new ArrayList<ArmEndEffectorState>();

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

        System.out.println(in.x + " " + in.y);

        Double XPosLimits[] = {0.0,10.0,10.0,0.0};
        Double YPosLimits[] = {0.0,0.0,10.0,10.0};

        ArmEndEffectorState clipPos = in;
        ArmEndEffectorState diff = new ArmEndEffectorState();
        ArmEndEffectorState p0 = new ArmEndEffectorState();
        ArmEndEffectorState p1 = new ArmEndEffectorState();
        ArmEndEffectorState cross = new ArmEndEffectorState();
        double m;
        double nearDist, p0Dist, crossDist;
        int crossCount = 0;
        int i;



        for ( i = 0; i < YPosLimits.length; i++) {

            // set up p0 and p1 for this segment
            p0.x = XPosLimits[i];
            p0.y = YPosLimits[i];
            if (i + 1 == YPosLimits.length) {
                p1.x = XPosLimits[0]; // close the boundary back onto the first point
                p1.y = YPosLimits[0];
            } else {
                p1.x = XPosLimits[i + 1];
                p1.y = YPosLimits[i + 1];
            }

            diff.x = p1.x - p0.x;
            diff.y = p1.y - p0.y;

            if (diff.y < SMALL_DIFF) {
                // horizontal line test (horizontal line does not cross a horizontal line)
                ;
            } else if (diff.x < SMALL_DIFF) {
                // vertical line
                if (in.x < Math.min(p0.x, p1.x) && in.y > Math.min(p0.y, p1.y) && in.y <= Math.max(p0.y, p1.y)) {
                    // crosses this vertical line
                    crossCount++;
                } else {
                    // does not cross the vertical line
                    ;
                }
            } else if (in.y > Math.min(p0.y, p1.y) && in.y <= Math.max(p0.y, p1.y)) {
                // boundary not horiz and not vert and in.y is in range of this segment
                cross.x = (in.y - p0.y) * diff.x / diff.y + p0.x;
                if (in.x < cross.x) {
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
            clipPos = in;
        } else {
            // in is outside of the fence so clip to nearest point on the boundary
            nearDist = LARGE_DIFF;

            for (i = 0; i < YPosLimits.length; i++) {

                // set up p0 and p1 for this segment
                p0.x = XPosLimits[i];
                p0.y = YPosLimits[i];
                if (i + 1 == YPosLimits.length) {
                    p1.x = XPosLimits[0]; // close the boundary back onto the first point
                    p1.y = YPosLimits[0];
                } else {
                    p1.x = XPosLimits[i + 1];
                    p1.y = YPosLimits[i + 1];
                }
                diff.x = p1.x - p0.x;
                diff.y = p1.y - p0.y;

                // calc p0 to in distance
                p0Dist = Math.sqrt(Math.pow(in.x - p0.x, 2) + Math.pow(in.y - p0.y, 2));
                if (diff.y < SMALL_DIFF) {
                    // horiz
                    cross.x = in.x;
                    cross.y = p0.y;
                    if (cross.x > Math.min(p0.x, p1.x) && cross.x <= Math.max(p0.x, p1.x)) {
                        // in range
                        crossDist = Math.abs(in.y - cross.y);
                    } else {
                        // out of range
                        crossDist = LARGE_DIFF;
                    }
                } else if (diff.x < SMALL_DIFF) {
                    // vert
                    cross.x = p0.x;
                    cross.y = in.y;
                    if (cross.y > Math.min(p0.y, p1.y) && cross.y <= Math.max(p0.y, p1.y)) {
                        // in range
                        crossDist = Math.abs(in.x - cross.x);
                    } else {
                        // out of range
                        crossDist = LARGE_DIFF;
                    }
                } else {
                    // not horiz or vert
                    m = diff.y / diff.x; // slope
                    cross.x = (m * p0.x - p0.y + in.x / m + in.y) / (m + 1.0 / m);
                    cross.y = m * cross.x - m * p0.x + p0.y;
                    if ((cross.x > Math.min(p0.x, p1.x) && cross.x <= Math.max(p0.x, p1.x)) &&
                            (cross.y > Math.min(p0.y, p1.y) && cross.y <= Math.max(p0.y, p1.y))) {
                        // in range
                        crossDist = Math.sqrt(Math.pow(in.x - cross.x, 2) + Math.pow(in.y - cross.y, 2));
                    } else {
                        // out of range
                        crossDist = LARGE_DIFF;
                    }
                }

                // check if the current distance is closest
                if (crossDist < p0Dist) {
                    if (crossDist < nearDist) {
                        // use this one
                        nearDist = crossDist;
                        clipPos = cross;
                    } else {
                        // leave nearest in place
                        ;
                    }
                } else {
                    if (p0Dist < nearDist) {
                        // use this one
                        nearDist = p0Dist;
                        clipPos = p0;
                    } else {
                        // leave nearest in place
                        ;
                    }
                }
            }
        }

        System.out.println(clipPos.x + " " + clipPos.y);
        return clipPos;

    }

}
