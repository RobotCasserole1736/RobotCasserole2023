package frc.robot.Arm;

import java.util.Arrays;

import javax.swing.text.html.HTMLDocument.HTMLReader.IsindexAction;

public class ArmSoftLimits {

    public ArmEndEffectorPos applyLimit(ArmEndEffectorPos in) {
       return null; // TODO - apply limits to the incoming position to fence it in
    }

    public static boolean robotSoftLimits(double wantedXPosArmImport, double wantedYPosArmImport, double actualXPosArmImport, double actualYPosArmImport, double[] importXLimits, double[] importYLimits) {
        // CURRENTLY A WIP IF YOU CRASH YOUR COMPUTER IT IS NOT MY FAULT
        // We will need to set the restriction variables to points on the limit lines 

        // This finds the slope of the line that the arm is following and the
        // restriction lines, uses that to see if they intersect,
        // and then sets the wanted position to the limit if it is outside the
        // restriction area.
        double[] restrictionXPoints = importXLimits; // This is where we declare the points for the restriction area.
                                                      // RESTRICTION LINES CAN NOT BE VERTICLE OR HORIZONTLE
        double[] restrictionYPoints = importYLimits;
        int loopForTestingSoftLimits = 0;

        double wantedYPosArm = wantedYPosArmImport; // These are temporary values, we will need to change them when we get the
                                  // simulation done.
        double wantedXPosArm = wantedXPosArmImport;
        double actualYPosArm = actualYPosArmImport;
        double actualXPosArm = actualXPosArmImport;
        boolean isItIntercepting;
        double YRestrictionPointOne;
        double XRestrictionPointOne;
        double YRestrictionPointTwo;
        double XRestrictionPointTwo;
        double slopeForAW;
        double slopeForRestriction; 
        double YInterceptForAW;
        double YInterceptForRestrictionLine;
        double WAAndRistrictionIntY;
        double  WAAndRistrictionIntX;
        double LengthOfAWLine;
        double LengthOfRistrictionLine;




for (loopForTestingSoftLimits = 0, isItIntercepting = false; (loopForTestingSoftLimits < restrictionXPoints.length) && (isItIntercepting = true); loopForTestingSoftLimits++ ) {



    System.out.println(loopForTestingSoftLimits + " isItIntercepting: " + isItIntercepting);

         YRestrictionPointOne = restrictionXPoints[loopForTestingSoftLimits];
         XRestrictionPointOne = restrictionYPoints[loopForTestingSoftLimits];

        System.out.println(loopForTestingSoftLimits + " YRestrictionPointOne: " + YRestrictionPointOne);
        System.out.println(loopForTestingSoftLimits + " XRestrictionPointOne: " + XRestrictionPointOne);



        if ((loopForTestingSoftLimits + 1) >= restrictionXPoints.length) {
            YRestrictionPointTwo = restrictionXPoints[0];
            XRestrictionPointTwo = restrictionYPoints[0];

        } else {

            YRestrictionPointTwo = restrictionXPoints[loopForTestingSoftLimits + 1];
            XRestrictionPointTwo = restrictionYPoints[loopForTestingSoftLimits + 1];

        }

        System.out.println(loopForTestingSoftLimits + " YRestrictionPointTwo: " + YRestrictionPointTwo);
        System.out.println(loopForTestingSoftLimits + " XRestrictionPointTwo: " + XRestrictionPointTwo);


         slopeForAW = (wantedYPosArm - actualYPosArm) / (wantedXPosArm - actualXPosArm); // finding slope of the line between the actual pos. and wanted pos.

        System.out.println("slopeForAW: " + slopeForAW);
        if ((XRestrictionPointOne - XRestrictionPointTwo) == 0) {
            slopeForRestriction = (YRestrictionPointOne - YRestrictionPointTwo) / (.00001);
        } else {
         slopeForRestriction = (YRestrictionPointOne - YRestrictionPointTwo) / (XRestrictionPointOne - XRestrictionPointTwo);
        }
        System.out.println( "slopeForRestriction: " + slopeForRestriction);
        if (Math.sqrt((slopeForAW - slopeForRestriction) * (slopeForAW - slopeForRestriction)) < 0.1 ) {

            System.out.println("Parrallel");

        } else {

            System.out.println("not Parrallel");
         YInterceptForAW = actualYPosArm - (slopeForAW * actualXPosArm); // finding y int for AW 
        System.out.println("YInterceptForAW: " + YInterceptForAW);

         YInterceptForRestrictionLine = YRestrictionPointOne - (slopeForRestriction * XRestrictionPointOne); // finding y int for restriction  
        System.out.println("YInterceptForRestrictionLine: " + YInterceptForRestrictionLine);

         WAAndRistrictionIntY = -1  * ((YInterceptForAW - YInterceptForRestrictionLine) / (slopeForAW - slopeForRestriction));
        System.out.println("WAAndRistrictionIntY: " + WAAndRistrictionIntY);

         WAAndRistrictionIntX = slopeForAW * WAAndRistrictionIntY - YInterceptForAW;
        System.out.println("WAAndRistrictionIntX: " + WAAndRistrictionIntX);
        
        LengthOfAWLine = Math.sqrt(((wantedXPosArm - actualXPosArm) * (wantedXPosArm - actualXPosArm)) + ((wantedYPosArm - actualYPosArm) * (wantedYPosArm - actualYPosArm)));
        System.out.println("LengthOfAWLine: " + LengthOfAWLine);

         LengthOfRistrictionLine = Math.sqrt(((actualXPosArm - WAAndRistrictionIntX) * (actualXPosArm - WAAndRistrictionIntX)) + ((actualYPosArm - WAAndRistrictionIntY) * (actualYPosArm - WAAndRistrictionIntY)));
        System.out.println("LengthOfRistrictionLine: " + LengthOfRistrictionLine);

        if (LengthOfAWLine > LengthOfRistrictionLine) {
            isItIntercepting = true;
System.out.println("It Worked");
        } else{

        }
        }        

}
    if (isItIntercepting == false) {
    return false;
} else {
    return true;
    }
}
}