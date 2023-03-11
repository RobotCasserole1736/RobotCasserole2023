package frc.robot.Autonomous.Events;

/*
 *******************************************************************************************
 * Copyright (C) FRC Team 1736 Robot Casserole - www.robotcasserole.org
 *******************************************************************************************
 *
 * This software is released under the MIT Licence - see the license.txt
 *  file in the root of this repo.
 *
 * Non-legally-binding statement from Team 1736:
 *  Thank you for taking the time to read through our software! We hope you
 *   find it educational and informative! 
 *  Please feel free to snag our software for your own use in whatever project
 *   you have going on right now! We'd love to be able to help out! Shoot us 
 *   any questions you may have, all our contact info should be on our website
 *   (listed above).
 *  If you happen to end up using our software to make money, that is wonderful!
 *   Robot Casserole is always looking for more sponsors, so we'd be very appreciative
 *   if you would consider donating to our club to help further STEM education.
 */

import frc.lib.AutoSequencer.AutoEvent;
import frc.robot.PoseTelemetry;
import frc.robot.Claw.ClawController;
import frc.robot.Drivetrain.DrivetrainControl;
import frc.robot.Drivetrain.DrivetrainPitchEstimator;
import frc.robot.Drivetrain.DrivetrainPitchEstimator.TiltState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Interface into the Casserole autonomous sequencer for a path-planned traversal. Simply wraps
 * path-planner functionality into the AutoEvent abstract class.
 */

public class AutoEventDriveFwdTillLevel extends AutoEvent {

    double duration = 0;
    double speed_mps = 0;

    double speedCmdRaw = 0;
    double speedCmdRateLimit = 0;

    DrivetrainControl dt_inst;
    DrivetrainPitchEstimator pe = DrivetrainPitchEstimator.getInstance();

    boolean hasTiltedUp = false;

    boolean level = false;

    Debouncer levelDebounce = new Debouncer(0.5, DebounceType.kRising);

    public AutoEventDriveFwdTillLevel(double max_duration, double speed_mps) {

        duration = max_duration;
        this.speed_mps = speed_mps;
        dt_inst = DrivetrainControl.getInstance();   
        
    }

    /**
     * On the first loop, calculates velocities needed to take the path specified. Later loops will
     * assign these velocities to the drivetrain at the proper time.
     */
    private double startTime = 0;

    public void userUpdate() {
        double curTime = (Timer.getFPGATimestamp()-startTime);

        
        /*if(pe.getCurTilt() == TiltState.NOSE_UP){
            hasTiltedUp = true;
        } */

        boolean debouncedLevelCondition = levelDebounce.calculate(pe.getCurTilt() == TiltState.LEVEL);

        if(debouncedLevelCondition){
            level = true;
        }

        if(level){
            dt_inst.setCmdRobotRelative(0.0, 0.0, 0.0, true);
        } else {
            dt_inst.setCmdRobotRelative(speed_mps, 0.0, 0.0, false);
            PoseTelemetry.getInstance().setDesiredPose(dt_inst.getCurEstPose());
        }

        /*if(hasTiltedUp) {
            if(debouncedLevelCondition || DriverStation.getMatchTime() <= 1){
                dt_inst.setCmdRobotRelative(0.0, 0.0, 0.0, true);

            } if (pe.getCurTilt() == TiltState.NOSE_UP){
                dt_inst.setCmdRobotRelative(speed_mps, 0.0, 0.0, false);

            } if (pe.getCurTilt() == TiltState.NOSE_DOWN){
                dt_inst.setCmdRobotRelative(-1 * speed_mps, 0.0, 0.0, false);

            }

        } else {
            //normal drive
            dt_inst.setCmdRobotRelative(speed_mps, 0.0, 0.0, false);

            //Populate desired pose from drivetrain - meh
            PoseTelemetry.getInstance().setDesiredPose(dt_inst.getCurEstPose());
        }*/

    }

    public void userForceStop() {
        ClawController.getInstance().setGrabCmd(false);
        ClawController.getInstance().setReleaseCmd(false);
       dt_inst.stop();
    }

    /**
     * Always returns true, since the routine should run as soon as it comes up in the list.
     */
    public boolean isTriggered() {
        return true; // we're always ready to go
    }

    /**
     * we never want it to end, so it will always return false
     */
    public boolean isDone() {

        return false;
    }

    @Override
    public void userStart() {
        startTime = Timer.getFPGATimestamp();
        hasTiltedUp = false;
    }

    public Pose2d getInitialPose() {
        return null;
    }


}