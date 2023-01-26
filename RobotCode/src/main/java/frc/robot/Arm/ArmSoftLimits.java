package frc.robot.Arm;



public class ArmSoftLimits {

    static final double SMALL_DIFF = 0.0001;
    static final double LARGE_DIFF = 9999999999999.9999;

    static ArmEndEffectorPos [] ArmLim = new ArmEndEffectorPos[] {};

    ArmLim[0].x = 0;
    ArmLim[0].y = 0;
    ArmLim[1].x = 10;
    ArmLim[1].y = 0;
    ArmLim[2].x = 10;
    ArmLim[2].y = 10;
    ArmLim[3].x = 0;
    ArmLim[3].y = 10;


    public ArmEndEffectorPos applyLimit(ArmEndEffectorPos in) {

        // This is the horizontal line method. Loop over each side and see if a horizontal line
        // drawn from the input point to the right crosses the fence volume once and only once.
        // If the input point is inside, then just return the input, else return the closest point
        // on the fence to the input point as clipped position.

        ArmEndEffectorPos clipPos = in;
        ArmEndEffectorPos diff;
        ArmEndEffectorPos nearest;
        ArmEndEffectorPos p0, p1;
        ArmEndEffectorPos cross;
        double m;
        double nearDist, p0Dist, crossDist;
        int crossCount = 0;
        

        for(int i=0; i<ArmLim.length; i++){

            // set up p0 and p1 for this segment
            p0 = ArmLim[i];
            if(i+1 == ArmLim.length){
                p1 = ArmLim[0]; // close the boundary back onto the first point
            } else {
                p1 = ArmLim[i+1];
            }

            diff.x = p1.x-p0.x;
            diff.y = p1.y-p0.y;

            if(diff.y < SMALL_DIFF){
                // horizontal line test (horizontal line does not cross a horizontal line)
                ;
            } else if(diff.x < SMALL_DIFF){
                // vertical line
                if(in.x < Math.min(p0.x,p1.x) && in.y > Math.min(p0.y,p1.y) && in.y <= Math.max(p0.y,p1.y)) {
                    // crosses this vertical line
                    crossCount++;
                } else {
                    // does not cross the vertical line
                    ;
                }
           } else if(in.y > Math.min(p0.y,p1.y) && in.y <= Math.max(p0.y,p1.y)){
                // boundary not horiz and not vert and in.y is in range of this segment
                cross.x = (in.y - p0.y)*diff.x/diff.y + p0.x;
                if(in.x < cross.x){
                    // horizontal line crosses the boundary
                    crossCount++;
           } else {
                // do nothing out of range
                ;
           }
        }

        if(crossCount == 1){
            // inside so no need to clip
            clipPos = in;
        } else {
            // in is outside of the fence so clip to nearest point on the boundary
            nearest = in;
            nearDist = LARGE_DIFF;

            for(i=0; i<ArmLim.length; i++){

                // set up p0 and p1 for this segment
                p0 = ArmLim[i];
                if(i+1 == ArmLim.length){
                    p1 = ArmLim[0]; // close the boundary back onto the first point
                } else {
                    p1 = ArmLim[i+1];
                }

                diff.x = p1.x-p0.x;
                diff.y = p1.y-p0.y;

                // calc p0 to in distance
                p0Dist = Math.sqrt(Math.pow(in.x-p0.x, 2)+Math.pow(in.y-p0.y, 2));
                if(diff.y < SMALL_DIFF){
                    // horiz
                    cross.x = in.x;
                    cross.y = p0.y;
                    if(cross.x > Math.min(p0.x,p1.x) && cross.x <= Math.max(p0.x,p1.x)){
                        // in range
                        crossDist = Math.abs(in.y-cross.y);
                    } else {
                        // out of range
                        crossDist = LARGE_DIFF;
                    }
                } else if(diff.x < SMALL_DIFF){
                    // vert
                    cross.x = p0.x;
                    cross.y = in.y;
                    if(cross.y > Math.min(p0.y,p1.y) && cross.y <= Math.max(p0.y,p1.y)){
                        // in range
                        crossDist = Math.abs(in.x-cross.x);
                    } else {
                        // out of range
                        crossDist = LARGE_DIFF;
                    }
                } else {
                    // not horiz or vert
                    m = diff.y/diff.x; // slope
                    cross.x = (m*p0.x-p0.y+in.x/m+in.y)/(m+1.0/m);
                    cross.y = m*cross.x-m*p0.x+p0.y;
                    if((cross.x > Math.min(p0.x,p1.x) && cross.x <= Math.max(p0.x,p1.x)) &&
                       (cross.y > Math.min(p0.y,p1.y) && cross.y <= Math.max(p0.y,p1.y))){
                        // in range
                        crossDist = Math.sqrt(Math.pow(in.x-cross.x, 2)+Math.pow(in.y-cross.y, 2));
                    } else {
                        // out of range
                        crossDist = LARGE_DIFF;
                    }
                }

                // check if the current distance is closest
                if(crossDist < p0Dist){
                    if(crossDist < nearDist){
                        // use this one
                        nearDist = crossDist;
                        clipPos = cross;
                    } else {
                        // leave nearest in place
                        ;
                    }
                } else {
                    if(p0Dist < nearDist){
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

        return clipPos;
    }
}
