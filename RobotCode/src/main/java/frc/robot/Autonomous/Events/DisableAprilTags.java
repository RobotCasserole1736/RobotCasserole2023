package frc.robot.Autonomous.Events;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.AutoSequencer.AutoEvent;
import frc.robot.Drivetrain.DrivetrainPoseEstimator;

public class DisableAprilTags extends AutoEvent {
	
	public DisableAprilTags() {
	}

	@Override
	public void userStart() {
        DrivetrainPoseEstimator.getInstance().useApriltags(false);
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
