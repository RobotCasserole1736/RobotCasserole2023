package frc.robot.Autonomous.Events;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.AutoSequencer.AutoEvent;
import frc.robot.ClawController;
import frc.robot.Arm.ArmNamedPosition;

/**
 * Selects Cone mode and returns immedeately
 */
public class AutoEventSetClawIntake extends AutoEvent {
	
	ArmNamedPosition posDes;
	double startTime = 0;
	final double MIN_DURATION_SEC = 0.25;
	final double MAX_DURATION_SEC = 1.0;
	
	public AutoEventSetClawIntake() {

	}

	@Override
	public void userStart() {
		startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void userUpdate() {
		// Just set control request the desired position
		ClawController.getInstance().setGrabCmd(true);
		ClawController.getInstance().setReleaseCmd(false);
	}

	@Override
	public void userForceStop() {
		// Force arm into safe state
		ClawController.getInstance().setGrabCmd(false);
		ClawController.getInstance().setReleaseCmd(false);
	}

	@Override
	public boolean isTriggered() {
		return true;
	}

	@Override
	public boolean isDone() {
		// Done = We're past the max time, or we're past the min time AND we detect a gamepiece
		var curTime = Timer.getFPGATimestamp() - startTime;
		boolean minTimeElapsed = curTime > MIN_DURATION_SEC;
		boolean maxTimeElapsed = curTime > MAX_DURATION_SEC;
		return maxTimeElapsed ||
		      (minTimeElapsed && ClawController.getInstance().hasGamepeice());
	}
}
