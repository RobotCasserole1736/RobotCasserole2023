package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.Constants;
import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;
import com.playingwithfusion.TimeOfFlight;


public class ClawController {
    Spark intakeWheelMotor;
    boolean curGrabCmd;
    boolean curReleaseCmd;
    Solenoid clawSolenoid;
    TimeOfFlight gamepieceDistSensor;

    @Signal
    boolean clawCloseCmd = true;

    @Signal
    double wheelMotorSpdCmd = 0;

    @Signal
    boolean gamepiecePresent = false;

    @Signal(units="in")
    double gamepieceDistSensorMeas = 0.0;

    Calibration cubePresentThresh;
    Calibration conePresentThresh;


    private static ClawController inst = null;

    public static synchronized ClawController getInstance() {
        if (inst == null)
            inst = new ClawController();
        return inst;
    }

    private ClawController() {
        intakeWheelMotor = new Spark(Constants.CLAW_INTAKE);
        clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.CLAW_SOLENOID);

        gamepieceDistSensor = new TimeOfFlight(Constants.GAMEPIECE_DIST_SENSOR_CANID);
        gamepieceDistSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        gamepieceDistSensor.setRangeOfInterest(6, 6, 10, 10);

        cubePresentThresh = new Calibration("Claw Cube Present Threshold", "in", 2);
        conePresentThresh = new Calibration("Claw Cone Present Threshold", "in", 3);
    }

    public void setReleaseCmd(boolean release) {
        curGrabCmd = release;
    }

    public void setGrabCmd(boolean grab) {
        curReleaseCmd = grab;
    }

    public void update() {
        var gpmm = GamepieceModeManager.getInstance();

        gamepieceDistSensorMeas = gamepieceDistSensor.getRange()/25.40; //Convert from mm to inches

        wheelMotorSpdCmd = 0.0;
        if (gpmm.isConeMode()) {

            if (curGrabCmd) {
                // close the claw and rotate wheels to suck in
                clawCloseCmd = true;
                wheelMotorSpdCmd = 1.0;
            } else if (curReleaseCmd) {
                // Open the claw
                clawCloseCmd = false;
            }

            gamepiecePresent = gamepieceDistSensorMeas < conePresentThresh.get();

        } else if (gpmm.isCubeMode()) {
            clawCloseCmd = false;

            if (curGrabCmd) {
                // Run the wheels to suck in
                wheelMotorSpdCmd = 1.0;
            } else if (curReleaseCmd) {
                // Run the wheels to eject
                wheelMotorSpdCmd = -1.0;
            }

            gamepiecePresent = gamepieceDistSensorMeas < cubePresentThresh.get();
        }

        intakeWheelMotor.set(wheelMotorSpdCmd);
        clawSolenoid.set(clawCloseCmd);
    }

    public boolean hasGamepeice() {
        return gamepiecePresent; 
    }

}