package frc.robot.Arm;

import java.util.Arrays;

public class ArmSoftLimits {

    public ArmEndEffectorPos applyLimit(ArmEndEffectorPos in){
        return null;// TODO - apply limits to the incoming position to fence it in
    }

    public boolean isLimited(){
        //TODO - return whether we're at alimit or not
// We will need to set the restriction variables to points on the limit lines - Kyle 
// It will return true if it is violating the soft barrier and false if it is not (i think?)
double[] restrictionXPoints = {1,1,3,3}; // should probably declare this somewhere else...
double[] restrictionYPoints = {1,2,3,4};
int loopForTestingSoftLimits = 0;

while(restrictionXPoints.length <= loopForTestingSoftLimits + 1) {

double restrictionPointOneX = restrictionXPoints[loopForTestingSoftLimits];
double restrictionPointOneY = restrictionYPoints[loopForTestingSoftLimits];
if (loopForTestingSoftLimits + 1 >= restrictionXPoints.length) {

double restrictionPointTwoX = restrictionXPoints[0];
double restrictionPointTwoY = restrictionYPoints[0];
} else {

double restrictionPointTwoX = restrictionXPoints[loopForTestingSoftLimits];
double restrictionPointTwoY = restrictionYPoints[loopForTestingSoftLimits];
}
double wantedYPosArm;
double wantedXPosArm;
double actualYPosArm;
double actualXPosArm;



double slopeForWantedLine = (wantedYPosArm - actualYPosArm) / (wantedXPosArm - actualXPosArm);
double slopeForRestrictionLine = (restrictionPointOneY - restrictionPointTwoY)/(restrictionPointOneX - restrictionPointTwoX);

 Boolean parrallelTestForLines = slopeForWantedLine == slopeForRestrictionLine;
 if (parrallelTestForLines == false) {
// finding the y intercepts for the two lines:
double yInterceptForWantedLine;
yInterceptForWantedLine = actualYPosArm - slopeForWantedLine * actualXPosArm;

double yInterceptForRestrictionLine;
yInterceptForRestrictionLine = restrictionPointOneY - slopeForRestrictionLine * restrictionPointOneX;

// finding where the two lines intercept
double wantedAndRestrictionLineInterceptXPos;
wantedAndRestrictionLineInterceptXPos = (0-1) * ((yInterceptForRestrictionLine - yInterceptForRestrictionLine) / (slopeForWantedLine - slopeForRestrictionLine));

double wantedAndRestrictionLineInterceptYPos;
wantedAndRestrictionLineInterceptYPos = slopeForWantedLine * wantedAndRestrictionLineInterceptXPos + yInterceptForWantedLine;

// declairing variables that it is forcing me to declair for some reason????????
double lengthForWantedLength = ((wantedXPosArm - actualXPosArm) * (wantedXPosArm - actualXPosArm))+ ((wantedYPosArm - actualYPosArm) * (wantedYPosArm - actualYPosArm));
double lengthForTestingLength = ((wantedAndRestrictionLineInterceptXPos - actualXPosArm) * (wantedAndRestrictionLineInterceptXPos - actualXPosArm)) + ((wantedAndRestrictionLineInterceptYPos - actualYPosArm) * (wantedAndRestrictionLineInterceptYPos - actualYPosArm));

if ( Math.sqrt(lengthForWantedLength) > Math.sqrt(lengthForTestingLength)) {
return true;

} else {
   loopForTestingSoftLimits = loopForTestingSoftLimits + 1; 
}

 
 }
}
return false;

}
    
}
