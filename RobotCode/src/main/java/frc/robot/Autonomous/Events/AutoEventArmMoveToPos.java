package frc.robot.Autonomous.Events;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.AutoSequencer.AutoEvent;
import frc.robot.Arm.ArmControl;
import frc.robot.Arm.ArmNamedPosition;

/**
 * Selects Cone mode and returns immedeately
 */
public class AutoEventArmMoveToPos extends AutoEvent {
	
	ArmNamedPosition posDes;
	double startTime = 0;
	final double MIN_DURATION_SEC = 0.5;
	Debouncer doneMovingArmDebouncer = new Debouncer(0.1, DebounceType.kRising);
	
	public AutoEventArmMoveToPos(ArmNamedPosition posDes) {
		this.posDes = posDes;
	}

	@Override
	public void userStart() {
		startTime = Timer.getFPGATimestamp();
	}

	@Override
	public void userUpdate() {
		// Just set control request the desired position
		ArmControl.getInstance().setOpCmds(0, 0, posDes, true, 0.0);
	}

	@Override
	public void userForceStop() {
		// Force arm into safe state
		ArmControl.getInstance().setOpCmds(0, 0, posDes, false, 0.0);
	}

	@Override
	public boolean isTriggered() {
		return true;
	}

	@Override
	public boolean isDone() {
		// Done = we've been commanding a position for a minimum number of seconds
		//        AND we've finished path planning
		var curTime = Timer.getFPGATimestamp() - startTime;
		boolean doneMovingDbnc = doneMovingArmDebouncer.calculate(!ArmControl.getInstance().isPathPlanning());
		return (doneMovingDbnc && curTime > MIN_DURATION_SEC);
	}
}
