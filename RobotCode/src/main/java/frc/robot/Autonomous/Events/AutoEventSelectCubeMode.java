package frc.robot.Autonomous.Events;

import frc.lib.AutoSequencer.AutoEvent;
import frc.robot.GamepieceModeManager;
import frc.robot.GamepieceModeManager.GamepieceMode;

/**
 * Selects Cone mode and returns immedeately
 */
public class AutoEventSelectCubeMode extends AutoEvent {
	
	
	public AutoEventSelectCubeMode() {

	}

	@Override
	public void userStart() {
		GamepieceModeManager.getInstance().setCurMode(GamepieceMode.CUBE);
	}

	@Override
	public void userUpdate() {
		return;
	}

	@Override
	public void userForceStop() {
		return;
	}

	@Override
	public boolean isTriggered() {
		return true;
	}

	@Override
	public boolean isDone() {
		return true;
	}
}
