package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.Constants;
import frc.lib.Signal.Annotations.Signal;

public class clawControl {
    Spark intakeWheels;
    boolean curEjectcmd;
    boolean curIntakecmd;
    Solenoid clawSolenoid; 

    @Signal
    boolean clawSolenoidCmd = true;



    // todo add open/close stuff

    public clawControl() {
        intakeWheels = new Spark(0);
       
        clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,Constants.CLAW_SOLENOID);

    }


    //TODO - use cube and code mode to change the meaning of eject and intake
    // cone = clamp and spin motors to intake, unclamp to eject
    // cube = spin motors for both, unclamped always.


    public void setEject(boolean eject) {
        curEjectcmd = eject;
    }

    public void setIntake(boolean intake) {
        curIntakecmd = intake;
    }

    public void update(GamepieceModeManager in) {

        if(in.isConeMode()){
            clawSolenoidCmd = true;
            //dashboard change?
        }
    
        if(in.isCubeMode()){
            clawSolenoidCmd = false;
            //dashboard change?
        }

        double speedCmd = 0;
        
        if (curEjectcmd) {
            speedCmd = -1;
        } else if (curIntakecmd){ 
            speedCmd = 1;
        }
        
        intakeWheels.set(speedCmd);
        clawSolenoid.set(clawSolenoidCmd);
    }

}