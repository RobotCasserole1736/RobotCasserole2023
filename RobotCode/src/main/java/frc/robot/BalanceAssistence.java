package frc.robot;

public class BalanceAssistence {
    
}

/*detect how the  robot is tilted (accelerometer)
    tilt = acceleration of gravity.

    theta = inverse tan(x-axis/y-axis) 

use said tilt to determine how far tilted the robot is

Tell motors to move a certain distance/speed to correct itself and position so the robot is level


get angle from accelerometer 
theta = inverse tan(x-axis/y-axis) 
degrees = theta calculation to degrees
if 0<degree<90 {
    upTilt = true;
    downTilt = false;
}
if -90<degree<0{
    downTilt = True;
    upTilt = false;
}

do {

} while (upTilt);
    balanceSpdCmd = fwdRevSlewLimiter.calculate((degrees/100) * Constants.MAX_FWD_REV_SPEED_MPS);
    **divide the degrees by some number, as long as it is between -1 and 1? Add some logic to stop if that number will be bigger than one? 
    **or alternately, just multiply by a constant that is different than the max possible command?
do {
    Make sure it's a negative value
} while (downTilt);

*/