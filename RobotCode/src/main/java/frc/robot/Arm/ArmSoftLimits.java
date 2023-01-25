package frc.robot.Arm;



public class ArmSoftLimits {

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
        ArmEndEffectorPos ends;
        ArmEndEffectorPos p0, p1;
        double m;
        int crossCount = 0;
        

        for(int i=0; i<ArmLim.length; i++){

            // set up p0 and p1 for this segment
            p0 = ArmLim[i];
            if(i == ArmLim.length-1){
                p1 = ArmLim[0]; // close the boundary back onto the first point
            } else {
                p1 = ArmLim[i+1];
            }

            diff.x = p1.x-p0.x;
            diff.y = p1.y-p0.y;

            if(diff.y != 0){ // horizontal line test (horizontal line does not cross a horizontal line)
                if(diff.x == 0){
                    // vertical line
                    if(p0.y < p1.y && in.y > p0.y && in.y <= p1.y){
                        // crosses this line
                        crossCount++;
                        // get the crossing point

                    } else if(p0.y > p1.y && in.y < p0.y && in.y >= p1.y){
                        // does not cross this line
                        // find closest point (perpendicular bisect or end point)
                    }
                } else if(((p0.y < p1.y && in.y > p0.y && in.y <= p1.y) ||
                          (p0.y > p1.y && in.y < p0.y && in.y >= p1.y)) &&
                          in.x < max(p0.x,p1.x)){
                    // crosses not horizontal and not vertical
                    crossCount++;

                } else {
                    // does not cross
                }
            }
        }

        return clipPos;
    }

    public boolean isLimited() {
        // TODO - return whether we're at alimit or not
        // We will need to set the restriction variables to points on the limit lines -
        // Kyle
        // It will return true if it is violating the soft barrier and false if it is
        // not (i think?)
        double[] restrictionXPoints = { 1, 1, 3, 3 }; // should probably declare this somewhere else...
        double[] restrictionYPoints = { 1, 2, 3, 4 };
        int loopForTestingSoftLimits = 0;

        while (restrictionXPoints.length <= loopForTestingSoftLimits + 1) {
            double restrictionPointTwoX;
            double restrictionPointTwoY;
            double restrictionPointOneX = restrictionXPoints[loopForTestingSoftLimits];
            double restrictionPointOneY = restrictionYPoints[loopForTestingSoftLimits];
            if (loopForTestingSoftLimits + 1 >= restrictionXPoints.length) {

                restrictionPointTwoX = restrictionXPoints[0];
                restrictionPointTwoY = restrictionYPoints[0];
            } else {

                restrictionPointTwoX = restrictionXPoints[loopForTestingSoftLimits];
                restrictionPointTwoY = restrictionYPoints[loopForTestingSoftLimits];
            }
            double wantedYPosArm = 0; //TODO - these are used before written currently
            double wantedXPosArm = 0;
            double actualYPosArm = 0;
            double actualXPosArm = 0;

            double slopeForWantedLine = (wantedYPosArm - actualYPosArm) / (wantedXPosArm - actualXPosArm);
            double slopeForRestrictionLine = (restrictionPointOneY - restrictionPointTwoY)
                    / (restrictionPointOneX - restrictionPointTwoX);

            Boolean parrallelTestForLines = slopeForWantedLine == slopeForRestrictionLine;
            if (parrallelTestForLines == false) {
                // finding the y intercepts for the two lines:
                double yInterceptForWantedLine;
                yInterceptForWantedLine = actualYPosArm - slopeForWantedLine * actualXPosArm;

                double yInterceptForRestrictionLine;
                yInterceptForRestrictionLine = restrictionPointOneY - slopeForRestrictionLine * restrictionPointOneX;

                // finding where the two lines intercept
                double wantedAndRestrictionLineInterceptXPos;
                wantedAndRestrictionLineInterceptXPos = (0 - 1)
                        * ((yInterceptForRestrictionLine - yInterceptForRestrictionLine)
                                / (slopeForWantedLine - slopeForRestrictionLine));

                double wantedAndRestrictionLineInterceptYPos;
                wantedAndRestrictionLineInterceptYPos = slopeForWantedLine * wantedAndRestrictionLineInterceptXPos
                        + yInterceptForWantedLine;

                // declairing variables that it is forcing me to declair for some reason????????
                double lengthForWantedLength = ((wantedXPosArm - actualXPosArm) * (wantedXPosArm - actualXPosArm))
                        + ((wantedYPosArm - actualYPosArm) * (wantedYPosArm - actualYPosArm));
                double lengthForTestingLength = ((wantedAndRestrictionLineInterceptXPos - actualXPosArm)
                        * (wantedAndRestrictionLineInterceptXPos - actualXPosArm))
                        + ((wantedAndRestrictionLineInterceptYPos - actualYPosArm)
                                * (wantedAndRestrictionLineInterceptYPos - actualYPosArm));

                if (Math.sqrt(lengthForWantedLength) > Math.sqrt(lengthForTestingLength)) {
                    return true;

                } else {
                    loopForTestingSoftLimits = loopForTestingSoftLimits + 1;
                }

            }
        }
        return false;

    }

}
