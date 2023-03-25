package frc.robot.Drivetrain.Camera;

import java.util.ArrayList;
import java.util.List;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.Constants;
import frc.lib.Faults.Fault;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.PoseTelemetry;
import frc.robot.Drivetrain.DrivetrainPitchState;
import frc.robot.Drivetrain.DrivetrainPitchState.TiltState;

public class PhotonCamWrapper {

    PhotonCamera cam;

    @Signal
    boolean isConnected;

    List<CameraPoseObservation> observations;

    final Transform3d robotToCam;

    Fault disconnectedFault;


    public PhotonCamWrapper(String cameraName, Transform3d robotToCam){
        this.cam = new PhotonCamera(cameraName);
        this.robotToCam = robotToCam;
        this.observations = new ArrayList<CameraPoseObservation>();
        disconnectedFault = new Fault(cameraName, "Camera not sending data");
    }

    public void update(Pose2d lastEstimate){

        disconnectedFault.set(!cam.isConnected());

        var res = cam.getLatestResult();
        double observationTime = res.getTimestampSeconds();

        PoseTelemetry.getInstance().setCamPose(this.cam.getName(), new Pose3d(lastEstimate).transformBy(robotToCam));

        List<PhotonTrackedTarget> tgtList = res.getTargets();

        observations = new ArrayList<CameraPoseObservation>();

        for(PhotonTrackedTarget t : tgtList){
            var id = t.getFiducialId();
            var targetPoseOpt = FieldTags.getInstance().lookup(id);
            if(targetPoseOpt.isPresent()){
                // We recognize the target id and know where it's at on the field
                var tgtPose = targetPoseOpt.get();

                // Snag both best and alternate cam to target estimates
                Transform3d ctotgt1 = t.getBestCameraToTarget(); 
                Transform3d ctotgt2 = t.getAlternateCameraToTarget(); 

                // Convert both cam to target estiamtes into bot pose ont he field
                Pose2d botPoseEst1 = toFieldPose(tgtPose, ctotgt1);
                Pose2d botPoseEst2 = toFieldPose(tgtPose, ctotgt2);

                //Pick the pose which is closer to where we were last at
                double err1 = botPoseEst1.minus(lastEstimate).getTranslation().getNorm();
                double err2 = botPoseEst2.minus(lastEstimate).getTranslation().getNorm();

                Pose2d bestEst;
                boolean useFirst = (err1 < err2); //Select the pose closest to our last pose estimate
                var amb = t.getPoseAmbiguity();
                boolean lowEnoughAmbiguity = amb < 0.2 && amb >= 0.0; // less than 0.2, and not -1

                boolean targetCloseEnoughToCamera = false;
                boolean poseIsOnField = false;
                boolean poseCloseEnoughToBot = false;

                if(useFirst){
                    bestEst = botPoseEst1;
                    targetCloseEnoughToCamera = ctotgt1.getTranslation().getNorm() < 0.5 * Constants.FIELD_LENGTH_M ;
                    poseIsOnField = poseIsOnField(botPoseEst1);
                    poseCloseEnoughToBot = botPoseEst1.minus(lastEstimate).getTranslation().getNorm() < Constants.FIELD_LENGTH_M;
                } else {
                    bestEst = botPoseEst2;
                    targetCloseEnoughToCamera = ctotgt2.getTranslation().getNorm() < 0.25 * Constants.FIELD_LENGTH_M ;
                    poseIsOnField = poseIsOnField(botPoseEst2);
                    poseCloseEnoughToBot = botPoseEst2.minus(lastEstimate).getTranslation().getNorm() < Constants.FIELD_LENGTH_M;
                }

                if(targetCloseEnoughToCamera && lowEnoughAmbiguity && poseIsOnField && poseCloseEnoughToBot){
                    // Target meets our filter criteria, add it.
                    observations.add(new CameraPoseObservation(observationTime, bestEst, 1.0)); 
                }

            } else {
                //TODO - handle case where we saw some tag other than 1 through 8
                DriverStation.reportError("Saw unknown tag ID " + Integer.toString(id), false);
            }

        }
    }

    private boolean poseIsOnField(Pose2d in){
        var trans = in.getTranslation();
        var x = trans.getX();
        var y = trans.getY();
        return x >= 0.0 && x <= Constants.FIELD_LENGTH_M && y >= 0.0 && y <= Constants.FIELD_WIDTH_M;
    }

    public List<CameraPoseObservation> getCurObservations(){
        return observations;
    }

    public int getCurTargetCount(){
        return observations.size();
    }

    private Pose2d toFieldPose(Pose3d tgtPose, Transform3d camToTarget){
        Pose3d camPose = tgtPose.transformBy(camToTarget.inverse());
        return camPose.transformBy(robotToCam.inverse()).toPose2d();   
    }

    public boolean isOnline(){
        return FieldTags.getInstance().isLoaded() && cam.isConnected();
    }

}
