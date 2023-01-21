package frc.robot.Drivetrain.Camera;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import frc.Constants;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.Signal.Annotations.Signal;

public class PhotonCamWrapper {

    PhotonCamera cam;

    @Signal
    boolean isConnected;

    List<CameraPoseObservation> observations;

    final Pose3d fieldPose = new Pose3d(); //Field-referenced orign

    final Transform3d robotToCam;


    public PhotonCamWrapper(String cameraName, Transform3d robotToCam){
        this.cam = new PhotonCamera(cameraName);
        this.robotToCam = robotToCam;
        this.observations = new ArrayList<CameraPoseObservation>();
    }

    public void update(Pose2d lastEstimate){

        var res = cam.getLatestResult();
        double observationTime = Timer.getFPGATimestamp() - res.getLatencyMillis();

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
                if(err1 < err2){
                    bestEst = botPoseEst1;
                } else {
                    bestEst = botPoseEst2;
                }

                //TODO - calculate a trustworthiness factor based on distance, ambiguity, etc.

                observations.add(new CameraPoseObservation(observationTime, bestEst, 1.0)); 
            } else {
                //TODO - handle case where we saw some tag other than 1 through 8
                DriverStation.reportError("Saw unknown tag ID " + Integer.toString(id), false);
            }

        }
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
