// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.lib.Calibration.CalWrangler;
import frc.lib.Faults.FaultWrangler;
import frc.lib.LoadMon.RIOLoadMonitor;
import frc.lib.LoadMon.SegmentTimeTracker;
import frc.lib.Logging.LogFileWrangler;
import frc.lib.Signal.SignalWrangler;
import frc.lib.Signal.Annotations.Signal;
import frc.lib.Webserver2.Webserver2;
import frc.robot.Arm.ArmControl;
import frc.robot.Arm.ArmNamedPosition;
import frc.robot.AutoDrive.AutoDrive;
import frc.robot.AutoDrive.AutoDrive.AutoDriveCmdState;
import frc.robot.Autonomous.Autonomous;
import frc.robot.Claw.ClawController;
import frc.robot.Drivetrain.DrivetrainControl;
import frc.robot.Drivetrain.DrivetrainPoseEstimator;
import frc.robot.GamepieceModeManager.GamepieceMode;
import frc.sim.RobotModel;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static double loopStartTime;

  ///////////////////////////////////////////////////////////////////
  // Instantiate new classes after here 
  // ...

  // Website utilities
  Webserver2 webserver;
  Dashboard db;
  CalWrangler cw;

  // Things
  RIOLoadMonitor loadMon;
  BatteryMonitor batMan;
  ArmControl ac;
  ClawController cc;
  DriverCamera dc;

  GamepieceModeManager mm;

  // Main Driver
  DriverInput di;
  OperatorInput oi; 

  //Drivetrain and drivetrain accessories
  DrivetrainControl dt;
  AutoDrive ad;

  // Autonomous Control Utilities
  Autonomous auto;
  PoseTelemetry pt;
  ArmTelemetry at;

  SegmentTimeTracker stt;

  @Signal(units = "sec")
  double mainLoopDuration;
  @Signal(units = "sec")
  double mainLoopPeriod;

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  // ... 
  // But before here
  ///////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////////////////////////////
  // Do one-time initilization here
  ///////////////////////////////////////////////////////////////////
  @Override
  public void robotInit() {
    stt = new SegmentTimeTracker("Robot.java", 0.25);
    stt.start();

    //Ensure we start with no heartbeat light
    FaultWrangler.getInstance().setHeartbeatActive(false);
    FaultWrangler.getInstance().setInit(true);


    // Disable default behavior of the live-window output manipulation logic
    // We've got our own and never use this anyway.
    LiveWindow.setEnabled(false);
    LiveWindow.disableAllTelemetry();
    stt.mark("LW Disable");

    // Start NT4 with logging
    NetworkTableInstance.getDefault().startServer();
    stt.mark("NT4");

    /* Init website utilties */
    webserver = new Webserver2();
    dc = DriverCamera.getInstance();
    stt.mark("Webserver2");

    cw = CalWrangler.getInstance();
    stt.mark("Cal Wrangler");

    loadMon = new RIOLoadMonitor();
    stt.mark("RIO Load Monitor");

    batMan = BatteryMonitor.getInstance();
    stt.mark("Battery Monitor");

    ac = ArmControl.getInstance();
    stt.mark("Arm Control");

    mm = GamepieceModeManager.getInstance();
    stt.mark("Gamepiece Manager");

    cc = ClawController.getInstance();
    stt.mark("Claw Control");


    di = new DriverInput(0);
    oi = new OperatorInput(1);
    stt.mark("Driver IO");

    dt = DrivetrainControl.getInstance();
    ad = new AutoDrive();
    stt.mark("Drivetrain Control");

    auto = Autonomous.getInstance();
    auto.loadSequencer();
    stt.mark("Autonomous");

    pt = PoseTelemetry.getInstance();
    at = ArmTelemetry.getInstance();
    stt.mark("Telemetry");

    db = new Dashboard(webserver);
    stt.mark("Dashboard");

    if(Robot.isSimulation()){
      simulationSetup();
    }
    syncSimPoseToEstimate();
    stt.mark("Simulation");

    SignalWrangler.getInstance().registerSignals(this);
    stt.mark("Signal Registration");

    webserver.startServer();
    stt.mark("Webserver Startup");

    PhotonCamera.setVersionCheckEnabled(false);
    stt.mark("Photonvision Config");

    System.gc();
    stt.mark("Post Init GC");

    System.out.println("Init Stats:");
    stt.end();

    //ITS ALIVE
    FaultWrangler.getInstance().setHeartbeatActive(true);

    

  }


  ///////////////////////////////////////////////////////////////////
  // Autonomous-Specific
  ///////////////////////////////////////////////////////////////////
  @Override
  public void autonomousInit() {

    LogFileWrangler.getInstance().syncLogFileNameToMatch();

    //Reset sequencer
    auto.reset();
    auto.startSequencer();

    // Ensure simulation resets to correct pose at the start of autonomous
    syncSimPoseToEstimate();

  }

  @Override
  public void autonomousPeriodic() {
    stt.start();
    loopStartTime = Timer.getFPGATimestamp();

    //Step the sequencer forward
    auto.update();
    stt.mark("Auto Update");

  }

  
  ///////////////////////////////////////////////////////////////////
  // Teleop-Specific
  ///////////////////////////////////////////////////////////////////
  @Override
  public void teleopInit() {
    DrivetrainPoseEstimator.getInstance().useApriltags(true);
  
  }

  @Override
  public void teleopPeriodic() {
    
    stt.start();
    loopStartTime = Timer.getFPGATimestamp();

    di.update();
    oi.update();
    stt.mark("Driver Input");

    /////////////////////////////////////
    // Drivetrain Input Mapping

    if(di.getSpinMoveCmd()){
      ad.setCmd(AutoDriveCmdState.DO_A_BARREL_ROLL);
    } else if (di.getDriveToCenterCmd()){
      ad.setCmd(AutoDriveCmdState.DRIVE_TO_CENTER);
    } else {
      ad.setCmd(AutoDriveCmdState.MANUAL);
    }

    ad.setManualCommands(di.getFwdRevCmd_mps(), di.getSideToSideCmd_mps(), di.getRotateCmd_rps(), !di.getRobotRelative(), di.getBracePositionCmd());
    ad.update();
    stt.mark("Auto Drive Calculation");

    if(oi.switchToConeModeCmd){
      mm.setCurMode(GamepieceMode.CONE);
    } else if (oi.switchToCubeModeCmd){
      mm.setCurMode(GamepieceMode.CUBE);
    } // else - leave mode unchanged

    var curOpPos = ArmNamedPosition.STOW;

    if(oi.armHighPosCmd){
      curOpPos = mm.isConeMode() ? ArmNamedPosition.CONE_HIGH : ArmNamedPosition.CUBE_HIGH;
    } else if(oi.armMidPosCmd){
      curOpPos = mm.isConeMode() ? ArmNamedPosition.CONE_MID : ArmNamedPosition.CUBE_MID;
    } else if(oi.armLowPosCmd){
      curOpPos = mm.isConeMode() ? ArmNamedPosition.CONE_LOW : ArmNamedPosition.CUBE_LOW;
    } else if(oi.armStowPosCmd){
      curOpPos = ArmNamedPosition.STOW;
    } else if (oi.armFloorTippedConePosCmd){
      curOpPos = ArmNamedPosition.FLOOR_TIPPED_CONE;
    } else if(oi.armShelfPosCmd){
      curOpPos = ArmNamedPosition.SHELF;
    }

    ac.setOpCmds(oi.curHorizontalCmd, oi.curVerticalCmd, curOpPos, oi.posCmdActive(), oi.armVertOffsetCmd);
    stt.mark("Arm Control");

    cc.setGrabCmd(di.getGrab() || oi.grabCmd);
    cc.setReleaseCmd(di.getRelease() || oi.releaseCmd);
    stt.mark("Claw Control");


    if(di.getOdoResetCmd()){
      //Reset pose estimate to angle 0, but at the same translation we're at
      var angle = (DriverStation.getAlliance() == Alliance.Red) ? 180.0 : 0.0;
      Pose2d newPose = new Pose2d(dt.getCurEstPose().getTranslation(), Rotation2d.fromDegrees(angle));
      dt.setKnownPose(newPose);
    }

    stt.mark("Human Input Mapping");

  }

  ///////////////////////////////////////////////////////////////////
  // Disabled-Specific
  ///////////////////////////////////////////////////////////////////
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    stt.start();
    loopStartTime = Timer.getFPGATimestamp();

    ac.setInactive();
    stt.mark("Arm Control");
    
    dt.calUpdate(false);
    stt.mark("Cal Updates");

    auto.sampleDashboardSelector();
    stt.mark("Auto Mode Update");
  }
  
  ///////////////////////////////////////////////////////////////////
  // Common Periodic updates
  ///////////////////////////////////////////////////////////////////
  @Override
  public void robotPeriodic() 
  {
    if(DriverStation.isTest() && !DriverStation.isDisabled()){
      dt.testUpdate();
    } else {
      dt.update();
    }
    stt.mark("Drivetrain");

    if(DriverStation.isTest() && !DriverStation.isDisabled()){
      ac.testUpdate();
    } else {
      ac.update();
    }
    stt.mark("Arm Controller");

    cc.update();
    stt.mark("Claw Controller");

    cw.update();
    stt.mark("Cal Wrangler");
    db.updateDriverView();
    stt.mark("Dashboard");
    dc.update();
    stt.mark("Driver Camera");
    telemetryUpdate();
    stt.mark("Telemetry");
    stt.end();

    mm.ledUpdate();

    //Peter says this will help eliminate our OOM issues when glass connects
    // and Peter is smart so we do what he says.
    NetworkTableInstance.getDefault().flushLocal();

    SmartDashboard.putNumber("SDB FPGATime", Timer.getFPGATimestamp());
  }

  private void telemetryUpdate(){
    double time = loopStartTime;

    dt.updateTelemetry();

    pt.setDesiredPose(dt.getCurDesiredPose());
    pt.setEstimatedPose(dt.getCurEstPose());
    
    pt.update(time);

    mainLoopDuration = stt.loopDurationSec;
    mainLoopPeriod = stt.loopPeriodSec;

    SignalWrangler.getInstance().sampleAllSignals(time);
  }

  ///////////////////////////////////////////////////////////////////
  // Test-Mode-Specific
  ///////////////////////////////////////////////////////////////////

  @Override
  public void testInit(){
    LiveWindow.setEnabled(false);
    // Tell the subsystems that care that we're entering test mode.
    dt.testInit();
  }

  @Override
  public void testPeriodic(){
    stt.start();
    loopStartTime = Timer.getFPGATimestamp();
    // Nothing special here, yet
  }

  ///////////////////////////////////////////////////////////////////
  // Simulation Support
  ///////////////////////////////////////////////////////////////////

  RobotModel plant;

  public void simulationSetup(){
    plant = new RobotModel();
    syncSimPoseToEstimate();
  }

  public void syncSimPoseToEstimate(){
    if(Robot.isSimulation()){
      plant.reset(dt.getCurEstPose());
    }
  }

  @Override
  public void simulationPeriodic(){
    plant.update(this.isDisabled());
    pt.setActualPose(plant.getCurActPose());
  }
}