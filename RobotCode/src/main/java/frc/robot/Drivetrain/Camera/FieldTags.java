package frc.robot.Drivetrain.Camera;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.Faults.Fault;

/**
 * Class to wrapper the actions of loading a .json file which defines all
 * expected apriltag locations on the field, handling file load errors 
 */
public class FieldTags {

    /* Singleton infrastructure */
    private static FieldTags instance;
    public static FieldTags getInstance() {
        if (instance == null) {
            instance = new FieldTags();
        }
        return instance;
    }

    AprilTagFieldLayout fieldTags; 

    Fault notLoadedFault = new Fault("Apriltag Pose", "Failed to load field definition file.");

    
    private FieldTags(){
        try {
            fieldTags = new AprilTagFieldLayout(Path.of(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "apriltagLayouts", "2023-chargedup.json"));
        } catch (IOException e) {
            DriverStation.reportError(null, null);
            fieldTags = null;
            e.printStackTrace();
        }
    }

    public Optional<Pose3d> lookup(int id){
        Optional<Pose3d> tmp = Optional.empty();
        if(isLoaded()){
            tmp = fieldTags.getTagPose(id);
            notLoadedFault.clearFault();
        }  else {
            notLoadedFault.reportFault();
        }
        return tmp;
    }

    public boolean isLoaded(){
        return (fieldTags != null);
    }   
    
}
