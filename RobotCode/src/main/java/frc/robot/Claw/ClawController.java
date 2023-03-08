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

    private final double CUBE_INTAKE_SPD = 0.55;
    private final double CUBE_HOLD_SPD = 0.2;
    private final double CUBE_EJECT_SPD = -0.35;

    private final double CONE_INTAKE_SPD = 0.65;
    private final double CONE_HOLD_SPD = 0.2;
    private final double CONE_EJECT_SPD = -0.3;

    @Signal
    boolean clawCloseCmd = true;

    @Signal
    double wheelMotorSpdCmd = 0;

    @Signal
    boolean gamepiecePresent = false;

    GamePieceDetector gpd;

    private static ClawController inst = null;

    public static synchronized ClawController getInstance() {
        if (inst == null)
            inst = new ClawController();
        return inst;
    }

    private ClawController() {
        intakeWheelMotor = new Spark(Constants.CLAW_INTAKE);
        clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.CLAW_SOLENOID);
        gpd = new GamePieceDetector();

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
        wheelMotorSpdCmd = 0.0; //default to zero speed
        if (gpmm.isConeMode()) {

            // Cone mode - claw closed unless releasing
            if(!gpd.hasGamepiece() && curGrabCmd){
                // No gamepiece, but actively intaking
                clawCloseCmd = true;
                wheelMotorSpdCmd = CONE_INTAKE_SPD;
            } else if (curReleaseCmd) {
                // releasing (with or without gamepiece)
                clawCloseCmd = false;
                wheelMotorSpdCmd = CONE_EJECT_SPD;
            } else if(gpd.hasGamepiece()){
                //Idle but with gamepiece
                clawCloseCmd = true;
                wheelMotorSpdCmd = CONE_HOLD_SPD;
            } else {
                //Idle but with no gamepiece
                clawCloseCmd = true;
                wheelMotorSpdCmd = 0.0;
            }

        } else if (gpmm.isCubeMode()) {

            //Cube mode - claw always open.
            clawCloseCmd = false;

            // Cone mode - claw closed unless releasing
            if(!gpd.hasGamepiece() && curGrabCmd){
                // No gamepiece, but actively intaking
                wheelMotorSpdCmd = CUBE_INTAKE_SPD;
            } else if (curReleaseCmd) {
                // releasing (with or without gamepiece)
                wheelMotorSpdCmd = CUBE_EJECT_SPD;
            } else if(gpd.hasGamepiece()){
                //Idle but with gamepiece
                wheelMotorSpdCmd = CUBE_HOLD_SPD;
            } else {
                //Idle but with no gamepiece
                wheelMotorSpdCmd = 0.0;
            }

        }

        //Electrically, positive command causes outward motion
        // while negative command causes inward motion
        intakeWheelMotor.set(-1 * wheelMotorSpdCmd); 

        // Claw pneumatic must be:
        //   off = closed
        //   on = opened
        // This is to ensure it can grip gamepieces
        // while disabled, before the match starts.
        clawSolenoid.set(!clawCloseCmd);
    }

    public boolean hasGamepiece() {
        return gpd.hasGamepiece();
    }

}