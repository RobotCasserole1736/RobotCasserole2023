package frc.robot;
import edu.wpi.first.wpilibj.PWM;
import frc.Constants;

/**
 * Class to track what gamepiece mode we're in.
 */
public class GamepieceModeManager {

    private final double PWM_CONE = -1.0;
    private final double PWM_CUBE = 0.0;

    /* Singleton infratructure*/
    private static GamepieceModeManager inst = null;
    public enum GamepieceMode{
        CUBE, //picking and placing cubes
        CONE; //picking and placing cones
    }

    private GamepieceMode curMode;
    PWM LEDPanelModeCtrl;

    public static synchronized GamepieceModeManager getInstance() {
        if (inst == null)
            inst = new GamepieceModeManager();
        return inst;
    }

    private GamepieceModeManager(){
        LEDPanelModeCtrl = new PWM(Constants.LED_MODE_PORT);

        setCurMode(GamepieceMode.CONE); //pick a default
    }

    public void setCurMode(GamepieceMode in){
        curMode = in;
        LEDPanelModeCtrl.setSpeed(curMode==GamepieceMode.CONE ? PWM_CONE : PWM_CUBE);
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
}
