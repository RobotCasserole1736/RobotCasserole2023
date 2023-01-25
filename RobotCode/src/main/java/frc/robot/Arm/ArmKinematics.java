package frc.robot.Arm;

public class ArmKinematics {

    //TODO - write some docs here somehow to describe the coordinate system we put on the board
    
    static ArmEndEffectorPos forward( ArmState in){
        return null;//TODO - return a real thing
    }   
        double InitialAngle_1 = (Math.PI*5)/3;
        double InitialAngle_2 = (Math.PI)/3;

        double BoomAngle = (Math.PI * 2) - InitialAngle_1;
        double StickAngle;
        {
        // Possible fix needed?
        if(InitialAngle_2 <= Math.PI){
            StickAngle = InitialAngle_2 - BoomAngle;
        } else {
            StickAngle = InitialAngle_2 + BoomAngle;
        }

        double InitialHeight = 60;
        //Starting height of the Boom-Stick system
        double BoomActual = 30.25;
        //Constant length of the Boom
        double StickActual = 32.5;
        //Same as above but for the Stick
        double BoomDisplacement = BoomActual*(Math.cos(BoomAngle));
        //X-Coordinate for the joining point between the Boom and the Stick
        double StickDisplacement = StickActual*(Math.sin(StickAngle));
        //Same as above but for the end of the Intake

        double ExBoom = Math.sqrt((BoomActual*BoomActual)-(BoomDisplacement*BoomDisplacement));
        //Pythagorean thereom but written in a weird way        
        double BoomHeight = InitialHeight - ExBoom;
        //Gives the Y-coordinate for the Boom
        double StickHeight;
        double ExStick = StickActual * (Math.sin(StickAngle));
        if(StickAngle <= Math.PI){
            StickHeight = BoomHeight + ExStick;
        } else {
            StickHeight = BoomHeight - ExStick;
        }
    }
        //Gives the height of the Stick that we need to add or remove
        

    static ArmState reverse( ArmEndEffectorPos in){
        //TODO - do reverse kinematics
        return null;//TODO - return a real thing
    }
    
}
