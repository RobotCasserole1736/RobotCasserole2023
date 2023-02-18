package frc.robot;

import frc.Constants;
import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

public class BalanceAssistence {

    Accelerometer accelerometer = new BuiltInAccelerometer();

    double xAccel; 
    double yAccel; 
    double tiltRads;
    double tiltDeg;

    boolean Tilt;
    boolean balanceCmd;

    @Signal (units="mps")
    double balanceSpdCmd;

    SlewRateLimiter balanceSlewLimiter;
    Calibration balanceSlewRate;


    public void setBalanceCmd(boolean balance) {
        balanceCmd = balance;
    }

   
    public void Update() { 

        xAccel = accelerometer.getX();
        yAccel = accelerometer.getY();
        tiltRads = Math.atan(xAccel/yAccel);
    
        // Get tilt in radians to degrees
        tiltDeg = Math.toDegrees(tiltRads);
        
        
        if(balanceCmd) { 
            //deciding if the robot has tilt enough to do any balancing
            if (5 < Math.abs(tiltDeg) && Math.abs(tiltDeg) < 90) {
                Tilt = true;
            } else {
                Tilt = false;
            }

            do {
                balanceSpdCmd = balanceSlewLimiter.calculate((tiltDeg/100) * Constants.MAX_FWD_REV_SPEED_MPS);
                //divide the degrees by some number, as long as it is between -1 and 1? Add some logic to stop if that number will be bigger than one? 
                //or alternately, just multiply by a constant that is different than the max possible command?
            } while (Tilt);

        } else {
            
            //What do we have to do here???!? If the balance command isn't on, then we dont want to do anything? 
            Tilt = false; 
            balanceSpdCmd = 0; 
        }
            
        
        if(balanceSlewRate.isChanged()) {
            balanceSlewLimiter = new SlewRateLimiter(balanceSlewRate.get());
        }
    }
    
}

/*detect how the  robot is tilted (accelerometer)

    xAccel = accelerometer.getX();
    yAccel = accelerometer.getY();
    tiltRads = Math.atan(xAccel/yAccel);

    // Get tilt in radians to degrees
    tiltDeg = Math.toDegrees(tiltRads);



    //deciding if the robot has tilt enough to do any balancing
    if 5<|tiltDeg|<90 {
        Tilt = True;
    } else {
        Tilt = False;
    }

    do {
        balanceSpdCmd = fwdRevSlewLimiter.calculate((tiltDeg/100) * Constants.MAX_FWD_REV_SPEED_MPS);
        **divide the degrees by some number, as long as it is between -1 and 1? Add some logic to stop if that number will be bigger than one? 
        **or alternately, just multiply by a constant that is different than the max possible command?
    } while (Tilt);
    
*/