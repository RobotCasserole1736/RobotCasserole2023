package frc.robot.Autonomous.Events;

import frc.lib.AutoSequencer.AutoEvent;
import frc.robot.GamepieceModeManager;
import frc.robot.GamepieceModeManager.GamepieceMode;

/**
 * Selects Cone mode and returns immedeately
 */
public class AutoEventSelectConeMode extends AutoEvent {
	
	
	public AutoEventSelectConeMode() {

	}

	@Override
	public void userStart() {
		GamepieceModeManager.getInstance().setCurMode(GamepieceMode.CONE);
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
