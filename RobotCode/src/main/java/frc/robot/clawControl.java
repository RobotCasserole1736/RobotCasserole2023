package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class clawControl {
    Spark intakeWheels;
    boolean curEjectcmd;
    boolean curIntakecmd;

    public clawControl() {
        intakeWheels = new Spark(0);
       
    }

    public void setEject(boolean eject) {
        curEjectcmd = eject;
    }

    public void setIntake(boolean intake) {
        curIntakecmd = intake;
    }

    public void update() {
        double speedCmd = 0;
        if (curEjectcmd) {
            speedCmd = -1;
        } else if (curIntakecmd){ 
            speedCmd = 1;
        }
        intakeWheels.set(speedCmd);
    }
}
