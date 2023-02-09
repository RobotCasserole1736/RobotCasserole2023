package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.Constants;
import frc.lib.Faults.FaultWrangler;
import frc.lib.Signal.SignalUtils;
import frc.lib.Signal.Annotations.Signal;
import frc.lib.Webserver2.Webserver2;
import frc.lib.Webserver2.DashboardConfig.DashboardConfig;
import frc.lib.Webserver2.DashboardConfig.SwerveStateTopicSet;
import frc.robot.Arm.ArmControl;
import frc.robot.Autonomous.Autonomous;
import frc.robot.Drivetrain.DrivetrainPoseEstimator;


public class Dashboard {

    @Signal(name = "db_visionTargetVisible")
    boolean visionTargetVisible;

    @Signal(name="db_armXPos")
    double armXPos;

    @Signal(name="db_armYPos")
    double armYPos;

    @Signal(name="db_pneumaticsPressure")
    double pnuemPressure;

    @Signal(name="db_dtSpeedLimited")
    boolean dtSpeedLimited;

    @Signal(name="db_armPathActive")
    boolean armPathActive;

    @Signal(name="db_cubeShape")
    boolean cubeShape;

    @Signal(name="db_triangleShape")
    boolean triangleShape;

    @Signal(name="db_isRedAlliance")
    boolean isRedAlliance;

    @Signal(name="db_isBlueAlliance")
    boolean isBlueAlliance;

    DashboardConfig d;

    public Dashboard (Webserver2 ws_in) {
        d = ws_in.dashboard;

        final double LEFT_COL = 17;
        final double CENTER_COL = 50;
        final double RIGHT_COL = 83;

        final double ROW1 = 15;
        final double ROW2 = 45;
        final double ROW3 = 55;
        final double ROW4 = 68;
        final double ROW5 = 80;

        d.addCamera("flcam", "http://" + Constants.cameraFrontLeftIP  + ":1182/stream.mjpg", LEFT_COL, ROW4, 0.60);
        d.addCamera("frcam", "http://" + Constants.cameraFrontRightIP + ":1182/stream.mjpg", RIGHT_COL, ROW4, 0.60);

        SwerveStateTopicSet[] topicList = new SwerveStateTopicSet[4];
        topicList[0] = new SwerveStateTopicSet("FL",0);
        topicList[1] = new SwerveStateTopicSet("FR",1);
        topicList[2] = new SwerveStateTopicSet("BL",2);
        topicList[3] = new SwerveStateTopicSet("BR",3);
        d.addSwerveState(topicList, "SwerveState", RIGHT_COL+3.5, ROW1, 0.8);
        
        d.addLineGauge(SignalUtils.nameToNT4ValueTopic("db_armXPos"), "Arm Extension", "in", -20, 50, -16, 48, CENTER_COL-11, ROW1-5, 1.0);
        d.addLineGauge(SignalUtils.nameToNT4ValueTopic("db_armYPos"), "Arm Height", "in", -5, 72, 0, 65, CENTER_COL-11, ROW1+5, 1.0);
        
        d.addCircularGauge(SignalUtils.nameToNT4ValueTopic("db_pneumaticsPressure"), "Pressure", "psi", 0, 140, 80, 130, CENTER_COL+10, ROW1, 1.0);

        d.addAutoChooser(Autonomous.getInstance().delayModeList, CENTER_COL, ROW2, 1.0);
        d.addAutoChooser(Autonomous.getInstance().mainModeList, CENTER_COL, ROW3, 1.0);

        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_masterCaution"),"Pneumatics uh-oh", "#FF0000", "icons/alert.svg", CENTER_COL+12, ROW4, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_dtSpeedLimited"),"DT Speed Limit", "#FFFF00", "icons/speed.svg", CENTER_COL-12, ROW4, 1.0);
        d.addIcon(FaultWrangler.getInstance().getFaultActiveTopic(), "Faults", "#FF0000", "icons/alert.svg", CENTER_COL-6, ROW4, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_visionTargetVisible"),"Vision Target Visible", "#00FF00", "icons/vision.svg", CENTER_COL, ROW4, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_armPathActive"),"Arm Path", "#00FFBB", "icons/autoAlign.svg", CENTER_COL+6, ROW4, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_isRedAlliance"),"Arm Path", "#FF3333", "icons/upperLimit.svg", CENTER_COL+12, ROW4, 1.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_isBlueAlliance"),"Arm Path", "#3333FF", "icons/lowerLimit.svg", CENTER_COL+18, ROW4, 1.0);
        
        
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_cubeShape"),"Cube", "#b515ef", "icons/cube.svg", LEFT_COL+5, ROW1, 2.0);
        d.addIcon(SignalUtils.nameToNT4ValueTopic("db_triangleShape"),"Triangle", "#FFFF00", "icons/triangle.svg", LEFT_COL-10, ROW1, 2.0);
        
        d.addText(FaultWrangler.getInstance().getFaultDescriptionTopic(),"Fault Description", CENTER_COL, ROW5, 1.0);

      }
    
      public void updateDriverView() {

        visionTargetVisible = DrivetrainPoseEstimator.getInstance().getVisionTargetsVisible();
        armXPos = Units.metersToInches(ArmTelemetry.getInstance().measPosX - Constants.WHEEL_BASE_HALF_LENGTH_M);
        armYPos = Units.metersToInches(ArmTelemetry.getInstance().measPosY);
        cubeShape = GamepieceModeManager.getInstance().isCubeMode();
        triangleShape = GamepieceModeManager.getInstance().isConeMode();
        armPathActive = ArmControl.getInstance().isPathPlanning();
        dtSpeedLimited = DriverStation.isTeleop() && ArmControl.getInstance().isExtended();
        pnuemPressure = PneumaticsSupplyControl.getInstance().getStoragePressure();
        isRedAlliance = DriverStation.getAlliance() == Alliance.Red;
        isBlueAlliance = DriverStation.getAlliance() == Alliance.Blue;
        
      }
    

}
