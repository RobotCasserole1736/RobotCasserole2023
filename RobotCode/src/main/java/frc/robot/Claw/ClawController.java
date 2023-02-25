package frc.robot.Claw;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.Constants;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.GamepieceModeManager;

public class ClawController {
    Spark intakeWheelMotor;
    boolean curGrabCmd;
    boolean curReleaseCmd;
    Solenoid clawSolenoid;

    @Signal
    boolean clawCloseCmd = true;

    @Signal
    double wheelMotorSpdCmd = 0;

    @Signal
    boolean gamepiecePresent = false;

    GampieceDetector gpd;

    private static ClawController inst = null;

    public static synchronized ClawController getInstance() {
        if (inst == null)
            inst = new ClawController();
        return inst;
    }

    private ClawController() {
        intakeWheelMotor = new Spark(Constants.CLAW_INTAKE);
        clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.CLAW_SOLENOID);
        gpd = new GampieceDetector();

    }

    public void setReleaseCmd(boolean release) {
        curGrabCmd = release;
    }

    public void setGrabCmd(boolean grab) {
        curReleaseCmd = grab;
    }

    public void update() {
        
        gpd.update();

        var gpmm = GamepieceModeManager.getInstance();

        // Main claw control logic. Drives the wheel motors and solenoid
        // to achieve the correct states per different gamepieces
        wheelMotorSpdCmd = 0.0;
        if (gpmm.isConeMode()) {

            if (curGrabCmd) {
                // close the claw and rotate wheels to suck in, until we have a gamepiece.
                clawCloseCmd = true;
                if(!gpd.hasGamepiece()){
                    wheelMotorSpdCmd = 1.0;
                } else {
                    wheelMotorSpdCmd = 0.0;
                }
            } else if (curReleaseCmd) {
                // Open the claw
                clawCloseCmd = false;
            }


        } else if (gpmm.isCubeMode()) {
            clawCloseCmd = false;

            if (curGrabCmd) {
                // Run the wheels to suck in until we have a gamepiece
                if(!gpd.hasGamepiece()){
                    wheelMotorSpdCmd = 1.0;
                } else {
                    wheelMotorSpdCmd = 0.0;
                }
            } else if (curReleaseCmd) {
                // Run the wheels to eject
                wheelMotorSpdCmd = -1.0;
            }

        }

        intakeWheelMotor.set(wheelMotorSpdCmd);
        clawSolenoid.set(clawCloseCmd);
    }

    public boolean hasGamepiece() {
        return gpd.hasGamepiece();
    }

}