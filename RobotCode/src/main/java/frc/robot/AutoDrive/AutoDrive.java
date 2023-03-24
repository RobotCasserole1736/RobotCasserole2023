package frc.robot.AutoDrive;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.Drivetrain.DrivetrainControl;

public class AutoDrive {

    //Team 1619-inspired driver assist module
    // Normally, the operator provides manual motion inputs
    // However, when commanded, this class will generate a trajectory on the fly
    // and run the trajectory as long as the operator commands it

    // Command State = what the operator wants the system to do.
     // Numbering of scoring positions is left to right from the perspective of the driver standing behind the glass
	public enum AutoDriveTargetPose{
		NONE(0), // operator has full control over drivetrain
		BLUE_9(9, new Translation2d(1.86, 0.50), true), 
		BLUE_8(8, new Translation2d(1.86, 1.05), true), 
		BLUE_7(7, new Translation2d(1.86, 1.65), true), 
		BLUE_6(6, new Translation2d(1.86, 2.19), true), 
		BLUE_5(5, new Translation2d(1.86, 2.72), true), 
		BLUE_4(4, new Translation2d(1.86, 3.28), true), 
		BLUE_3(3, new Translation2d(1.86, 3.90), true), 
		BLUE_2(2, new Translation2d(1.86, 4.42), true), 
		BLUE_1(1, new Translation2d(1.86, 4.96), true), 
        RED_1(11, new Translation2d(14.65, 0.50), false), 
		RED_2(12, new Translation2d(14.65, 1.05), false), 
		RED_3(13, new Translation2d(14.65, 1.65), false), 
		RED_4(14, new Translation2d(14.65, 2.19), false), 
		RED_5(15, new Translation2d(14.65, 2.72), false), 
		RED_6(16, new Translation2d(14.65, 3.28), false), 
		RED_7(17, new Translation2d(14.65, 3.90), false), 
		RED_8(18, new Translation2d(14.65, 4.42), false), 
		RED_9(19, new Translation2d(14.65, 4.96), false);

		public final int value;
        public final Pose2d tgtPose;

		private AutoDriveTargetPose(int value) {
			this.value = value;
            this.tgtPose = null;
		}

		private AutoDriveTargetPose(int value, Translation2d tgt, boolean isBlue) {
			this.value = value;
            this.tgtPose = new Pose2d(tgt, Rotation2d.fromDegrees(isBlue? 180.0 : 0.0));
		}

		public int toInt() {
			return this.value;
		}
	}

    // State of the auto-drive algorithm's state machine
    public enum AutoDriveState{ 
		MANUAL(0), // Passing driver commands through to the drivetrain
		GENERATING_TRAJECTORY(1), //Driver commands still active, but a trajectory is being generated in the background
		RUNNING_TRAJECTORY(2); //Trajectory is available, and is currently being run.
		public final int value;

		private AutoDriveState(int value) {
			this.value = value;
		}

		public int toInt() {
			return this.value;
		}
	}

    double manualFwdRevCmd = 0;
    double manualStrafeCmd = 0;
    double manualRotateCmd = 0;
    boolean manualFieldRelativeCmd = false;
    boolean bracePosition = false;

    AutoDriveTargetPose curCmd = AutoDriveTargetPose.NONE;
    AutoDriveTargetPose prevCmd = AutoDriveTargetPose.NONE;

    AutoDriveState curState = AutoDriveState.MANUAL;
    AutoDriveState prevState = AutoDriveState.MANUAL;

    DynamicSwerveTrajectoryGenerator curTraj = null;

    //Holonomic specific - the heading (direction we point the front of the robot in)
    // is independent of the velocity direction of the trajectory
    Rotation2d startHeading;
    Rotation2d endHeading;

    @Signal
    double curAutoCmdRotDeg;
    @Signal
    double curAutoCmdRotVelDegPerSec;

    public AutoDrive(){

    }

    public void setManualCommands(double fwdRevCmd, double strafeCmd, double rotateCmd, boolean fieldRelativeCmd, boolean bracePosition){
        this.manualFwdRevCmd = fwdRevCmd;
        this.manualStrafeCmd = strafeCmd;
        this.manualRotateCmd = rotateCmd;
        this.manualFieldRelativeCmd = fieldRelativeCmd;
        this.bracePosition = bracePosition;
    }

    public void setCmd(AutoDriveTargetPose cmd){
        curCmd = cmd;
    }

    public void update(){
        DrivetrainControl dt = DrivetrainControl.getInstance();

        // Update state machine
        if(curCmd == AutoDriveTargetPose.NONE){
            //Manual always takes prescedence
            curState = AutoDriveState.MANUAL;
        } else if ( curCmd != prevCmd ){
            //Command has changed, initiate a trajectory calculation

            System.out.println("[AutoDrive] New Auto Drive Trajectory generation starting...");
            
            DynamicSwerveWaypointSet waypoints = new DynamicSwerveWaypointSet();

            //TODO - we assume the start of the trajectory is where we're at now, not where
            // we're actually at once the trajectory starts. todo... maybe mark the
            // current time and advance the time of the trajectory by the calculation time?

            //Most of these are reasonable defaults and get overwritten based on current command.
            waypoints.start = dt.getCurEstPose();
            waypoints.startRot = dt.getCurEstPose().getRotation();
            waypoints.interiorWaypoints = new ArrayList<Translation2d>();

            // Pick waypoint ends based on the command
            waypoints.end = curCmd.tgtPose;
            waypoints.endRot = waypoints.end.getRotation();

            // Start the dynamic generation
            curTraj = new DynamicSwerveTrajectoryGenerator();
            curTraj.startGeneration(waypoints);            
            curState = AutoDriveState.GENERATING_TRAJECTORY;
            
        } else if (curState == AutoDriveState.GENERATING_TRAJECTORY){
            if(curTraj.isReady()){
                System.out.println("[AutoDrive] Starting trajectory...");
                curTraj.startTrajectory();
                curState = AutoDriveState.RUNNING_TRAJECTORY;
            }
        } else if(curState == AutoDriveState.RUNNING_TRAJECTORY){
            // No transitino until driver releases command
        }


        // Send outputs to the drivetrain
        if(curState == AutoDriveState.MANUAL || curState == AutoDriveState.GENERATING_TRAJECTORY){
            if(manualFieldRelativeCmd){
                dt.setCmdFieldRelative(manualFwdRevCmd, manualStrafeCmd, manualRotateCmd, bracePosition);
            } else {
                dt.setCmdRobotRelative(manualFwdRevCmd, manualStrafeCmd, manualRotateCmd, bracePosition);
            }
        } else if(curState == AutoDriveState.RUNNING_TRAJECTORY) {
            var curCmd = curTraj.getCurCmd();
            dt.setCmdTrajectory(curCmd);

            //Debug signals
            curAutoCmdRotVelDegPerSec = curCmd.desAngVel.getDegrees();
            curAutoCmdRotDeg = curCmd.desAngle.getDegrees();

        } else {
            dt.setCmdRobotRelative(0, 0, 0, false);
        }


        // Update previous 
        prevState = curState;
        prevCmd = curCmd;
    }
    
}
