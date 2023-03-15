package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.Constants;
import frc.lib.Signal.Annotations.Signal;
import frc.lib.Util.Mechanism2DMarker;
import frc.lib.Util.Mechanism2DPolygon;
import frc.robot.Arm.ArmEndEffectorState;
import frc.robot.Arm.Path.ArmPath;
import frc.robot.Arm.ArmAngularState;

/**
 * Class to wrapper all the logic to create a ... uhh... very complex
 * Mechanism2d widget which represents our arm's state. And lots of other stuff.
 */
public class ArmTelemetry {

    /* Singleton infratructure */
    private static ArmTelemetry inst = null;

    boolean ENABLE_TELEMETRY = Robot.isSimulation();
    //boolean ENABLE_TELEMETRY = true;

    public static synchronized ArmTelemetry getInstance() {
        if (inst == null)
            inst = new ArmTelemetry();
        return inst;
    }

    // All Dimensions in meters
    // https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023FieldDrawings-CHARGEDUPSpecific.pdf
    // page 10 for most of these
    private final double LEFT_MARGIN = Units.inchesToMeters(15);
    private final double LOW_GOAL = LEFT_MARGIN + Constants.WHEEL_BASE_HALF_LENGTH_M + Constants.BUMPER_THICKNESS_M; // account
                                                                                                                     // for
                                                                                                                     // bumpers
                                                                                                                     // &
                                                                                                                     // frame
    private final double MID_GOAL = LOW_GOAL + Units.inchesToMeters(22.7);
    private final double MID_GOAL_HEIGHT = Units.inchesToMeters(34.0);
    private final double HIGH_GOAL = LOW_GOAL + Units.inchesToMeters(39.37);
    private final double HIGH_GOAL_HEIGHT = Units.inchesToMeters(46.0);

    // Overall mechanism 2d which everything gets drawn on
    private final Mechanism2d m_mech2d = new Mechanism2d(HIGH_GOAL + LEFT_MARGIN, Units.feetToMeters(6.0));

    // Fixed structures which always exist on the drawing
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", LEFT_MARGIN,
            Constants.ARM_BOOM_MOUNT_HIEGHT);
    private final MechanismLigament2d m_arm_tower = m_armPivot.append(
            new MechanismLigament2d("ArmTower", Constants.ARM_BOOM_MOUNT_HIEGHT, -90, 5, new Color8Bit(Color.kSilver)));

    private final MechanismRoot2d midNodeHome = m_mech2d.getRoot("Mid Node", MID_GOAL, 0);
    private final MechanismLigament2d MidNode = midNodeHome
            .append(new MechanismLigament2d("Mid Cone Node", MID_GOAL_HEIGHT, 90, 10, new Color8Bit(Color.kWhite)));

    private final MechanismRoot2d highNodeHome = m_mech2d.getRoot("High Node", HIGH_GOAL, 0);
    private final MechanismLigament2d HighNode = highNodeHome
            .append(new MechanismLigament2d("High Cone Node", HIGH_GOAL_HEIGHT, 90, 10, new Color8Bit(Color.kWhite)));

    private final MechanismRoot2d bumperRoot = m_mech2d.getRoot("BumperRoot", 0, 0.01);
    private final MechanismLigament2d bumpers = bumperRoot
            .append(new MechanismLigament2d("Bumpers", LOW_GOAL, 0, 50, new Color8Bit(Color.kRed)));
    private final MechanismLigament2d nodeBase = bumpers
            .append(new MechanismLigament2d("Node Base", HIGH_GOAL - LOW_GOAL, 0, 100, new Color8Bit(Color.kWhite)));

    // Marker to show where the end effector is desired to be at at all times
    private final Mechanism2DMarker desEndEffPos = new Mechanism2DMarker(m_mech2d, "de", 0.02);

    // Smooth line which shows the most recently calculated path, and its
    // constituent waypoints
    private final Mechanism2DPolygon desPath = new Mechanism2DPolygon(m_mech2d, "dp",
            new ArrayList<Translation2d>());
            //Decimate down to just 8 waypoints, hopefully nothing more than that needed
    private final List<Mechanism2DMarker> desPathWaypointMarkers = new ArrayList<Mechanism2DMarker>(8);
                                                                                                        
                                                                                                        
                                                                                                        
                                                                                                        
                                                                                                        
                                                                                                        

    // Outline which shows where the soft limits logic should be limiting motion to
    private final Mechanism2DPolygon softLimits = new Mechanism2DPolygon(m_mech2d, "sl",
            new ArrayList<Translation2d>());

    // Actual arm position ligaments to show the arm's location from
    // simulation. This won't be available on the real robot.
    private final MechanismLigament2d m_boom_ligament_act = m_armPivot.append(
            new MechanismLigament2d(
                    "Arm Boom Act",
                    Constants.ARM_BOOM_LENGTH,
                    -90,
                    3,
                    new Color8Bit(Color.kSteelBlue)));

    private final MechanismLigament2d m_stick_ligament_act = m_boom_ligament_act.append(
            new MechanismLigament2d(
                    "Arm Stick Act",
                    Constants.ARM_STICK_LENGTH,
                    Units.radiansToDegrees(0.0),
                    3,
                    new Color8Bit(Color.kSteelBlue)));

    // Ligaments to show the arm's location as measured by sensors.
    // This will be available on the real robot.
    private final MechanismLigament2d m_boom_ligament_meas = m_armPivot.append(
            new MechanismLigament2d(
                    "Arm Boom Meas",
                    Constants.ARM_BOOM_LENGTH,
                    -90,
                    5,
                    new Color8Bit(Color.kRed)));

    private final MechanismLigament2d m_stick_ligament_meas = m_boom_ligament_meas.append(
            new MechanismLigament2d(
                    "Arm Stick Meas",
                    Constants.ARM_STICK_LENGTH,
                    Units.radiansToDegrees(0.0),
                    5,
                    new Color8Bit(Color.kRed)));

    // Partial ligaments to show the desired angles of each joint
    private final MechanismLigament2d m_boom_ligament_des = m_armPivot.append(
            new MechanismLigament2d(
                    "Arm Boom Des",
                    Constants.ARM_BOOM_LENGTH / 3.0,
                    -90,
                    10,
                    new Color8Bit(Color.kLimeGreen)));

    private final MechanismLigament2d m_stick_ligament_des = m_boom_ligament_meas.append(
            new MechanismLigament2d(
                    "Arm Stick Des",
                    Constants.ARM_STICK_LENGTH / 3.0,
                    Units.radiansToDegrees(0.0),
                    10,
                    new Color8Bit(Color.kLimeGreen)));

    // Temporary debug variables to put key measurements into NT
    @Signal
    double desPosX;
    @Signal
    double measPosX;
    @Signal
    double desPosY;
    @Signal
    double measPosY;
    @Signal
    boolean desReflex;

    private ArmTelemetry() {
        if(ENABLE_TELEMETRY){
            // Put Mechanism 2d to SmartDashboard
            SmartDashboard.putData("Arm", m_mech2d);
        }

        // Init all waypoint markers
        for (int i = 0; i < 8; i++) {
            desPathWaypointMarkers.add(new Mechanism2DMarker(m_mech2d, "dpm" + Integer.toString(i), 0.015));
        }

        // configure styles
        for (Mechanism2DMarker marker : desPathWaypointMarkers) {
            marker.setVisible(false);
            marker.setStyle(new Color8Bit(Color.kLightCyan), 1);
        }
        desEndEffPos.setStyle(new Color8Bit(Color.kLimeGreen), 2);
        desPath.setStyle(new Color8Bit(Color.kCyan), 1);
        softLimits.setStyle(new Color8Bit(Color.kRed), 1);

        if (Robot.isReal()) {
            // Effectively hide the Act ligament if we're on a real robot
            m_boom_ligament_act.setLength(0);
            m_stick_ligament_act.setLength(0);
        }
    }

    // Given an ArmPath object, this will create a polygon evenly sampled
    // along the path and draw it.
    public void setDesPath(ArmPath path) {
        var pathPoly = new ArrayList<Translation2d>();

        int NUM_INTERNAL_SEGMENTS = 8;
        double deltaT = path.getDurationSec() / NUM_INTERNAL_SEGMENTS;

        for (double time = 0.0; time < path.getDurationSec(); time += deltaT) {
            var point = path.sample(time);
            pathPoly.add(new Translation2d(point.x + LEFT_MARGIN, point.y));
        }

        var point = path.sample(path.getDurationSec());
        pathPoly.add(new Translation2d(point.x + LEFT_MARGIN, point.y));

        desPath.setPolygon(pathPoly);

        var waypoints = path.getWaypoints();
        for (int i = 0; i < desPathWaypointMarkers.size(); i++) {
            var marker = desPathWaypointMarkers.get(i);
            if (i < waypoints.size()) {
                var waypoint = waypoints.get(i);
                marker.setLocation(waypoint.plus(new Translation2d(LEFT_MARGIN, 0)));
                marker.setVisible(true);
            } else {
                marker.setVisible(false);
            }
        }

    }

    // Updates where the soft limits polygon is drawn on the display
    public void setSoftLimits(List<Translation2d> softLimitPoly) {
        softLimits.setOrigin(new Translation2d(LEFT_MARGIN, 0));
        softLimits.setPolygon(softLimitPoly);
    }

    // Updates the actual (simulated) position of the arm
    public void setActual(double boomAngleDeg, double stickAngleDeg) {
        m_boom_ligament_act.setAngle(boomAngleDeg);
        m_stick_ligament_act.setAngle(stickAngleDeg);
    }

    // Updates the control logic's desired angular position and end effector
    // position
    // as drawn on the mechanism 2d
    public void setDesired(ArmEndEffectorState desPos, ArmAngularState desArmState) {
        desEndEffPos.setLocation(new Translation2d(desPos.x + LEFT_MARGIN, desPos.y));
        m_boom_ligament_des.setAngle(desArmState.boomAngleDeg);
        m_stick_ligament_des.setAngle(desArmState.stickAngleDeg);

        desPosX = desPos.x;
        desPosY = desPos.y;
        desReflex = desPos.isReflex;
    }

    // Update the control logic's measured angular and end effector position
    public void setMeasured(ArmEndEffectorState measPos, ArmAngularState measArmState) {
        m_boom_ligament_meas.setAngle(measArmState.boomAngleDeg);
        m_stick_ligament_meas.setAngle(measArmState.stickAngleDeg);

        measPosX = measPos.x;
        measPosY = measPos.y;
    }

}
