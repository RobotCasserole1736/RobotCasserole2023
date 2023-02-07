package frc.robot.Arm;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.ArmTelemetry;

import frc.robot.Arm.ArmEndEffectorState;

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

        double XLimits[] = {1.4};
double YLimits[] = {1,4};

        double XPos = in.x;
        double YPos = in.x;

double correctXPos;
double correctYPos;

double veloX = in.xvel;
double veloY = in.yvel;


if (XPos > XLimits[0] && YPos > YLimits[0] && XPos < XLimits[1] && YPos < YLimits[1]) {
    correctXPos = XPos;
    correctYPos = YPos;

  veloX = in.xvel;
  veloY = in.yvel;

} else {

   if (XPos < XLimits[0]) {
 veloX = 0;
    if (YPos < YLimits[0]) {
        veloY = 0;
        correctXPos = XLimits[0];
        correctYPos = YLimits[0];
    } else if (YPos > YLimits[1]) {
        veloY = 0;
        correctXPos = XLimits[0];
        correctYPos = YLimits[1];
    } else {
        correctXPos = XLimits[0];
        correctYPos = YPos;
    }

   } else if (XPos > XLimits[1]) {
    veloX = 0;
    if (YPos < YLimits[0]) {
        veloY = 0;
        correctXPos = XLimits[1];
        correctYPos = YLimits[0];
    } else if (YPos > YLimits[1]) {
        veloY = 0;
        correctXPos = XLimits[1];
        correctYPos = YLimits[1];
    } else {
        correctXPos = XLimits[1];
        correctYPos = YPos;
    }

   } else {

    if (YPos < YLimits[0]) {
        veloY = 0;
        correctXPos = XPos;
        correctYPos = YLimits[0];
    } else if (YPos > YLimits[1]) {
        veloY = 0;
        correctXPos = XPos;
        correctYPos = YLimits[1];
    } else {
        correctXPos = XPos;
        correctYPos = YPos;
 
   }


   }
}

System.out.println(correctXPos);
System.out.println(correctYPos);

return new ArmEndEffectorState(XPos,YPos,veloX,veloY,in.reflexFrac); 



        
        // TODO - apply limits to the incoming position to fence it in
    }

    public boolean robotSoftLimits(double XPosArmImport, double YPosArmImport, double[] importXLimits, double[] importYLimits) {
 
double XLimits[] = importXLimits;
double YLimits[] = importYLimits;


double XPos = XPosArmImport;
double YPos = YPosArmImport;

double correctXPos;
double correctYPos;

if (XPos > XLimits[0] && YPos > YLimits[0] && XPos < XLimits[1] && YPos < YLimits[1]) {
    correctXPos = XPos;
    correctYPos = YPos;



} else {

   if (XPos < XLimits[0]) {

    if (YPos < YLimits[0]) {
        correctXPos = XLimits[0];
        correctYPos = YLimits[0];
    } else if (YPos > YLimits[1]) {
        correctXPos = XLimits[0];
        correctYPos = YLimits[1];
    } else {
        correctXPos = XLimits[0];
        correctYPos = YPos;
    }

   } else if (XPos > XLimits[1]) {

    if (YPos < YLimits[0]) {
        correctXPos = XLimits[1];
        correctYPos = YLimits[0];
    } else if (YPos > YLimits[1]) {
        correctXPos = XLimits[1];
        correctYPos = YLimits[1];
    } else {
        correctXPos = XLimits[1];
        correctYPos = YPos;
    }

   } else {

    if (YPos < YLimits[0]) {
        correctXPos = XPos;
        correctYPos = YLimits[0];
    } else if (YPos > YLimits[1]) {
        correctXPos = XPos;
        correctYPos = YLimits[1];
    } else {
        correctXPos = XPos;
        correctYPos = YPos;
 
   }


   }
}

System.out.println(correctXPos);
System.out.println(correctYPos);

return false; 
}
}