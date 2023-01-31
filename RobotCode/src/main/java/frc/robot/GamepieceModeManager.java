package frc.robot;
import edu.wpi.first.wpilibj.PWM;
import frc.Constants;

public class GamepieceModeManager {

    /* Singleton infratructure*/
    private static GamepieceModeManager inst = null;
    public enum GamepieceMode{
        CUBE,
        CONE;
    }
    
    // Mignt not need this
    public enum LEDGameModePatterns{
        purpleCube,
        yellowCone;
    }

    private GamepieceMode curMode;
    PWM LEDPanelModeCtrl;

    public static synchronized GamepieceModeManager getInstance() {
        if (inst == null)
            inst = new GamepieceModeManager();
        return inst;
    }

    private GamepieceModeManager(){
        curMode = GamepieceMode.CONE; //pick a default
        LEDPanelModeCtrl = new PWM(Constants.LED_MODE_PORT);
    }

    public void setCurMode(GamepieceMode in){
        curMode = in;
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
            curMode = GamepieceMode.CUBE;
        } else {
            curMode = GamepieceMode.CONE;
        }
    }
}
