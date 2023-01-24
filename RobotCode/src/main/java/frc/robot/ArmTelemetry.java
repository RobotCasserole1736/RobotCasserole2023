package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.Constants;
import frc.robot.Arm.ArmEndEffectorPos;

public class ArmTelemetry {

    /* Singleton infratructure*/
    private static ArmTelemetry inst = null;
    public static synchronized ArmTelemetry getInstance() {
        if (inst == null)
            inst = new ArmTelemetry();
        return inst;
    }

    //All Dimensions in meters
    // https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023FieldDrawings-CHARGEDUPSpecific.pdf page 10 for most of these
    private final double LEFT_MARGIN = Units.inchesToMeters(15);
    private final double LOW_GOAL = LEFT_MARGIN + Constants.WHEEL_BASE_HALF_LENGTH_M + Units.inchesToMeters(5.0); // account for bumpers & frame
    private final double MID_GOAL  = LOW_GOAL + Units.inchesToMeters(22.7);
    private final double MID_GOAL_HEIGHT = Units.inchesToMeters(34.0);
    private final double HIGH_GOAL =  LOW_GOAL + Units.inchesToMeters(39.37);
    private final double HIGH_GOAL_HEIGHT = Units.inchesToMeters(46.0);

    private final Mechanism2d m_mech2d = new Mechanism2d(HIGH_GOAL + LEFT_MARGIN, Units.feetToMeters(6.0));

    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", LEFT_MARGIN, Constants.ARM_BOOM_MOUNT_HIEGHT);
    private final MechanismLigament2d m_arm_tower =
        m_armPivot.append(new MechanismLigament2d("ArmTower", Constants.ARM_BOOM_MOUNT_HIEGHT, -90, 10, new Color8Bit(Color.kSilver)));
  
    private final MechanismRoot2d midNodeHome = m_mech2d.getRoot("Mid Node", MID_GOAL, 0);
    private final MechanismLigament2d MidNode = midNodeHome.append(new MechanismLigament2d("Mid Cone Node", MID_GOAL_HEIGHT, 90, 10, new Color8Bit(Color.kWhite)));

    private final MechanismRoot2d highNodeHome = m_mech2d.getRoot("High Node", HIGH_GOAL, 0);
    private final MechanismLigament2d HighNode = highNodeHome.append(new MechanismLigament2d("High Cone Node", HIGH_GOAL_HEIGHT, 90, 10, new Color8Bit(Color.kWhite)));

    private final MechanismRoot2d bumperRoot = m_mech2d.getRoot("BumperRoot", 0, 0.01);
    private final MechanismLigament2d bumpers = bumperRoot.append(new MechanismLigament2d("Bumpers", LOW_GOAL, 0, 50, new Color8Bit(Color.kRed)));
    private final MechanismLigament2d nodeBase = bumpers.append(new MechanismLigament2d("Node Base", HIGH_GOAL - LOW_GOAL, 0, 100, new Color8Bit(Color.kWhite)));

    private final MechanismRoot2d desEndPosRoot = m_mech2d.getRoot("DesEndEffPos", 1, 1);
    private final MechanismLigament2d desEndPosMarker1 = desEndPosRoot.append(new MechanismLigament2d("EndPosMarker1", 0.02, 0, 20, new Color8Bit(Color.kYellowGreen)));
    private final MechanismLigament2d desEndPosMarker2 = desEndPosRoot.append(new MechanismLigament2d("EndPosMarker2", 0.02, 180, 20, new Color8Bit(Color.kYellowGreen)));


    private final MechanismLigament2d m_boom_ligament =
        m_armPivot.append(
              new MechanismLigament2d(
                "Arm Boom",
                Constants.ARM_BOOM_LENGTH, 
                -90, 
                10, 
                new Color8Bit(Color.kSkyBlue)));

    private final MechanismLigament2d m_stick_ligament =
        m_boom_ligament.append(
            new MechanismLigament2d(
                "Arm Stick",
                Constants.ARM_STICK_LENGTH,
                Units.radiansToDegrees(0.0),
                10,
                new Color8Bit(Color.kBlue)));

    private ArmTelemetry(){
        // Put Mechanism 2d to SmartDashboard
        SmartDashboard.putData("Arm", m_mech2d);
    }

    public void setActual(double boomAngleDeg, double stickAngleDeg){
        m_boom_ligament.setAngle(boomAngleDeg);
        m_stick_ligament.setAngle(stickAngleDeg);
    }
    public void setDesired(ArmEndEffectorPos des, double upperAngleDeg, double lowerAngleDeg){
        desEndPosRoot.setPosition(des.x, des.y);
    }
    public void setMeasured(double upperAngleDeg, double lowerAngleDeg){

    }
    
}
