package frc.sim;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionSystem {

    ////////////////////////////////////////////////////////////////
    // Simulated Vision System.

    //Todo pick up nolans stuff ere

    //TODO - need to pull latest photonvision for this
    //SimVisionSystem simVisionFront =
    //new SimVisionSystem(
    //        "FRONT_CAM",
    //        camDiagFOV,
    //        camPitch,
    //        Constants.robotToFrontCameraTrans.inverse(),
    //        camHeightOffGround,
    //        maxLEDRange,
    //        camResolutionWidth,
    //        camResolutionHeight,
    //        minTargetArea);

    
    //SimVisionSystem simVisionRear =
    //new SimVisionSystem(
    //        "REAR_CAM",
    //        camDiagFOV,
    //        camPitch,
    //        Constants.robotToRearCameraTrans.inverse(),
    //        camHeightOffGround,
    //        maxLEDRange,
    //        camResolutionWidth,
    //        camResolutionHeight,
    //        minTargetArea);


    public VisionSystem(){
        //TODO - pick up latest photonvision to support this
        //simVisionFront.addSimVisionTarget(new SimVisionTarget(farTargetPose, targetHeight/2, targetWidth, targetHeight));
        //simVisionRear.addSimVisionTarget(new SimVisionTarget(farTargetPose, targetHeight/2, targetWidth, targetHeight));
    }

    public void update(Pose2d curPose){
        //TODO - pick up latest photonvision to support this
        //simVisionFront.processFrame(curPose);
        //simVisionRear.processFrame(curPose);
    }
    
}
