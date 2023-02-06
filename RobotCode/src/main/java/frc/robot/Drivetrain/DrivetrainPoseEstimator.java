package frc.robot.Drivetrain;


import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import frc.Constants;
import frc.hardwareWrappers.Gyro.WrapperedGyro;
import frc.hardwareWrappers.Gyro.WrapperedGyro.GyroType;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.PoseTelemetry;
import frc.robot.Drivetrain.Camera.FieldTags;
import frc.robot.Drivetrain.Camera.PhotonCamWrapper;

/**
 * The pose estimator takes in measurements from drivetrain encoders and cameras
 * and puts them into wpilib's kalman filter. In turn, this will return a best guess
 * at where our robot is at on the field
 */
public class DrivetrainPoseEstimator {

    /* Singleton infrastructure */
    private static DrivetrainPoseEstimator instance;
    public static DrivetrainPoseEstimator getInstance() {
        if (instance == null) {
            instance = new DrivetrainPoseEstimator();
        }
        return instance;
    }

    // Current best-estimate of where we're at on the field.
    // Start it out at an assumed default pose (though, this is very likely
    // to get overridden by an autonomous routine)
    Pose2d curEstPose = new Pose2d(Constants.DFLT_START_POSE.getTranslation(), Constants.DFLT_START_POSE.getRotation());

    // Gyroscope returns a measruement of the robot's angle relative to the field
    WrapperedGyro gyro;

    // WPILib pose estimator
    SwerveDrivePoseEstimator m_poseEstimator;

    // List of all photonvision cameras on the robot which gather
    // information about apriltags on the field
    List<PhotonCamWrapper> cams = new ArrayList<PhotonCamWrapper>();

    //Trustworthiness of the internal model of how motors should be moving
    // Measured in expected standard deviation (meters of position and degrees of rotation)
    Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

    //Trustworthiness of the vision system
    // Measured in expected standard deviation (meters of position and degrees of rotation)
    Matrix<N3, N1>  visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);


    // Mostly for debug or dashboard purposes, plot
    // the robot's current speed on the field.
    @Signal(units = "ft/sec")
    double curSpeed = 0;

    private DrivetrainPoseEstimator(){

        // Invoke the fieldTags getInstance at least once
        // to be sure the feild definition file is loaded.
        FieldTags.getInstance();

        // Create all vision processing cameras and add them to the list of cameras
        cams.add(new PhotonCamWrapper("FRONT_LEFT_CAM", Constants.robotToFrontLeftCameraTrans)); 
        cams.add(new PhotonCamWrapper("FRONT_RIGHT_CAM", Constants.robotToFrontRightCameraTrans)); 
        cams.add(new PhotonCamWrapper("REAR_CAM", Constants.robotToRearCameraTrans)); 

        gyro = new WrapperedGyro(GyroType.ADXRS453);

        //Temp default - will populate with real valeus in the resetPosition method
        SwerveModulePosition[] initialStates = {new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()};

        m_poseEstimator = new SwerveDrivePoseEstimator(
            Constants.m_kinematics,
            new Rotation2d(),
            initialStates,
            new Pose2d(),
            stateStdDevs,
            visionMeasurementStdDevs
            );

    }

    /**
     * Snap update the estimator to a known pose.
     * @param in known pose
     */
    public void setKnownPose(Pose2d in){
        DrivetrainControl.getInstance().resetWheelEncoders();
        gyro.reset(in.getRotation().getRadians());
        var states = DrivetrainControl.getInstance().getModuleActualPositions();
        
        m_poseEstimator.resetPosition(in.getRotation(), states, in);
        curEstPose = in;
    }

    /**
     * 
     * @return the current best estimate of the robot's position on the field
     */
    public Pose2d getEstPose(){ return curEstPose; }

    public void update(){

        // Handle gyro-related update tasks
        gyro.update();

        //Based on gyro and measured module speeds and positions, estimate where our robot should have moved to.
        SwerveModulePosition[] positions = DrivetrainControl.getInstance().getModuleActualPositions();
        Pose2d prevEstPose = curEstPose;
        curEstPose = m_poseEstimator.update(getGyroHeading(), positions);

        //Calculate a "speedometer" velocity in ft/sec
        Transform2d deltaPose = new Transform2d(prevEstPose, curEstPose);
        curSpeed = Units.metersToFeet(deltaPose.getTranslation().getNorm()) / Constants.Ts;

        PoseTelemetry.getInstance().clearVisionPoses();

        for(var cam : cams){
            cam.update(getEstPose());
            for(var obs : cam.getCurObservations()){
                m_poseEstimator.addVisionMeasurement(obs.estFieldPose, obs.time, visionMeasurementStdDevs.times(1.0/obs.trustworthiness));
                PoseTelemetry.getInstance().addVisionPose("Tmp", obs.estFieldPose);
            }
        }

    }

    /**
     * 
     * @return representation of the robot's heading as measured by the gyroscope
     */
    public Rotation2d getGyroHeading(){
        return gyro.getRotation2d();
    }

    /**
     * 
     * @return the current linear speed of the robot in feet per second
     */
    public double getSpeedFtpSec(){
        return curSpeed;
    }

    /**
     * 
     * @return true if at least one camera sees one target, false otherwise
     */
    public boolean getVisionTargetsVisible(){
        for(var cam:cams){
            if(cam.getCurTargetCount() > 0){
                return true;
            }
        }
        return false;
    }


}