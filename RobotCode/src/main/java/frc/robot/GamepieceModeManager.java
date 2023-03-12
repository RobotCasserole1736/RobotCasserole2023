package frc.robot;
import edu.wpi.first.wpilibj.PWM;
import frc.Constants;
import frc.lib.Signal.Annotations.Signal;

/**
 * Class to track what gamepiece mode we're in.
 */
public class GamepieceModeManager {

    private final double PWM_CONE = -1.0;
    private final double PWM_CUBE = 0.0;
    private final double PWM_BLINK = 1.0;

    @Signal
    boolean ledBlinkCmd = false;

    /* Singleton infratructure*/
    private static GamepieceModeManager inst = null;
    public enum GamepieceMode{
        CUBE, //picking and placing cubes
        CONE; //picking and placing cones
    }

    private GamepieceMode curMode;
    PWM LEDPanelModeCtrl;
    PWM LEDStripsModeCtrl;

    public static synchronized GamepieceModeManager getInstance() {
        if (inst == null)
            inst = new GamepieceModeManager();
        return inst;
    }

    private GamepieceModeManager(){
        LEDPanelModeCtrl = new PWM(Constants.LED_MODE_PORT);
        LEDStripsModeCtrl = new PWM(Constants.LED_STRIP_PORT);

        setCurMode(GamepieceMode.CONE); //pick a default
    }

    public void setCurMode(GamepieceMode in){
        curMode = in;

    }  

    public void ledUpdate() {
        
        if(ledBlinkCmd){
            LEDPanelModeCtrl.setSpeed(PWM_BLINK);
            LEDStripsModeCtrl.setSpeed(PWM_BLINK);
        } else if (curMode==GamepieceMode.CONE) {
            LEDPanelModeCtrl.setSpeed(PWM_CONE);
            LEDStripsModeCtrl.setSpeed(PWM_CONE);
        } else if (curMode==GamepieceMode.CUBE) {
            LEDPanelModeCtrl.setSpeed(PWM_CUBE);
            LEDStripsModeCtrl.setSpeed(PWM_CUBE);
        }
    }

    public GamepieceMode getCurMode(){
        return curMode;
    }

    public boolean isConeMode(){
        return (curMode == GamepieceMode.CONE);
    }

    public boolean isCubeMode(){
        return (curMode == GamepieceMode.CUBE);
    }

    public void toggleMode(){
        if(isConeMode()){
            setCurMode(GamepieceMode.CUBE);
        } else {
            setCurMode(GamepieceMode.CONE);
        }
    }

    public void setBlinkMode(boolean in) {
        ledBlinkCmd = in;
    }
}
