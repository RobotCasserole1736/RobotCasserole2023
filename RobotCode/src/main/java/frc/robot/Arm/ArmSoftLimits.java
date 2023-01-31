package frc.robot.Arm;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.ArmTelemetry;

public class ArmSoftLimits {

    double[] restrictionXPoints = { 1, 1, 4, 4 }; // This is where we declare the points for the restriction area.
    // RESTRICTION LINES CAN NOT BE VERTICLE OR HORIZONTLE
    double[] restrictionYPoints = { 1, 4, 4, 1 };

    public ArmSoftLimits(){

        updateTelemetry();
    }

    public void updateTelemetry(){
        //for now - send over the restriction points one-time to telemetry
        var softLimitPoly = new ArrayList<Translation2d>();
        for(int idx = 0; idx < restrictionXPoints.length; idx++){
            //Pack x/y coordinates into Translation2d's
            var x = restrictionXPoints[idx];
            var y = restrictionYPoints[idx];
            softLimitPoly.add(new Translation2d(x,y));
        }
        //Close back to first point
        var x = restrictionXPoints[0];
        var y = restrictionYPoints[0];
        softLimitPoly.add(new Translation2d(x,y));
        
        ArmTelemetry.getInstance().setSoftLimits(softLimitPoly);
    }

    public ArmEndEffectorState applyLimit(ArmEndEffectorState in) {
        return null;// TODO - apply limits to the incoming position to fence it in
    }

    public boolean robotSoftLimits(double wantedXPosArmImport, double wantedYPosArmImport, double actualXPosArmImport, double actualYPosArmImport, double[] importXLimits, double[] importYLimits) {
        // CURRENTLY A WIP IF YOU CRASH YOUR COMPUTER IT IS NOT MY FAULT
        // We will need to set the restriction variables to points on the limit lines -
        // Kyle

        // This finds the slope of the line that the arm is following and the
        // restriction lines, uses that to see if they intersect,
        // and then sets the wanted position to the limit if it is outside the
        // restriction area.
       
        int loopForTestingSoftLimits = 0;

        double wantedYPosArm = wantedYPosArmImport; // These are temporary values, we will need to change them when we get the
                                  // simulation done.
        double wantedXPosArm = wantedXPosArmImport;
        double actualYPosArm = actualYPosArmImport;
        double actualXPosArm = actualXPosArmImport;

        while (restrictionXPoints.length >= loopForTestingSoftLimits + 1) { // this loop will repeat for each side

            double restrictionPointOneX = restrictionXPoints[loopForTestingSoftLimits];// this sets the restriction
                                                                                       // points to the relevant value
            double restrictionPointOneY = restrictionYPoints[loopForTestingSoftLimits];
            double restrictionPointTwoX; // this declares some variables that the if statement will use.
            double restrictionPointTwoY;

            if (loopForTestingSoftLimits + 1 >= restrictionXPoints.length) { // this if statement checks to see if the
                                                                             // realivant value for the restrictions is
                                                                             // the last one on the list
                restrictionPointTwoX = restrictionXPoints[0]; // if is the last one, it chooses the first value
                restrictionPointTwoY = restrictionYPoints[0];
            } else {
                restrictionPointTwoX = restrictionXPoints[loopForTestingSoftLimits]; // if not it chooses the following
                                                                                     // value
                restrictionPointTwoY = restrictionYPoints[loopForTestingSoftLimits];
            }
            double slopeForWantedLine = (wantedYPosArm - actualYPosArm) / (wantedXPosArm - actualXPosArm); // this finds the slope for the line we are traviling on & the slope for the restriction line
            double slopeForRestrictionLine;

            if (restrictionPointTwoX - restrictionPointOneX == 0) { // my attempt to try to prevent restrictionangle var from being NaN :/ I dont know if it works yet
                slopeForRestrictionLine = (restrictionPointTwoY - restrictionPointOneY) / (0.01);
            } else {

                slopeForRestrictionLine = (restrictionPointTwoY - restrictionPointOneY)
                        / (restrictionPointTwoX - restrictionPointOneX);
            }
            Boolean parrallelTestForLines = 0.1 > Math.sqrt(
                    (slopeForWantedLine - slopeForRestrictionLine) * (slopeForWantedLine - slopeForRestrictionLine)); // this sets a boolean var. to true if the two lines are close enough to parrallel
            if (parrallelTestForLines == false) { // if the two are not parrallel then it executes the following code:
                // finding the y intercepts for the two lines:
                double yInterceptForWantedLine;
                yInterceptForWantedLine = actualYPosArm - slopeForWantedLine * actualXPosArm;

                double yInterceptForRestrictionLine;
                yInterceptForRestrictionLine = restrictionPointOneY - slopeForRestrictionLine * restrictionPointOneX;

                // finding the x pos where the two lines intercept
                double wantedAndRestrictionLineInterceptXPos;
                wantedAndRestrictionLineInterceptXPos = (0 - 1)
                        * ((yInterceptForRestrictionLine - yInterceptForRestrictionLine)
                                / (slopeForWantedLine - slopeForRestrictionLine));
                // finding the y pos where the two lines intercept
                double wantedAndRestrictionLineInterceptYPos;
                wantedAndRestrictionLineInterceptYPos = slopeForWantedLine * wantedAndRestrictionLineInterceptXPos
                        + yInterceptForWantedLine;

                // finding the lengths of the two lines.
                double lengthForWantedLength = ((wantedXPosArm - actualXPosArm) * (wantedXPosArm - actualXPosArm))
                        + ((wantedYPosArm - actualYPosArm) * (wantedYPosArm - actualYPosArm));
                double lengthForTestingLength = ((wantedAndRestrictionLineInterceptXPos - actualXPosArm)
                        * (wantedAndRestrictionLineInterceptXPos - actualXPosArm))
                        + ((wantedAndRestrictionLineInterceptYPos - actualYPosArm)
                                * (wantedAndRestrictionLineInterceptYPos - actualYPosArm));

                if (Math.sqrt(lengthForWantedLength) > Math.sqrt(lengthForTestingLength)) { // testing to see if the line between the wanted point and actual point is longer than the length of the line between the intercection point and the actual pos.
                    return true; // it is intercepting

                } else {
                    loopForTestingSoftLimits = loopForTestingSoftLimits + 1; // adds 1 to the number of times that the
                                                                             // loop has run.
                }
            }
        }
        return false; // it is not
    }
}
