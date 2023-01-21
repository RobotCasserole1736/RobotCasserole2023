package frc.robot;

public class GamepieceModeManager {

    /* Singleton infratructure*/
    private static GamepieceModeManager inst = null;
    public static synchronized GamepieceModeManager getInstance() {
        if (inst == null)
            inst = new GamepieceModeManager();
        return inst;
    }

    private GamepieceModeManager(){
        curMode = GamepieceMode.CONE;//pick a default
    }
    
    private GamepieceMode curMode;

    public enum GamepieceMode{
        CUBE,
        CONE;
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
}
