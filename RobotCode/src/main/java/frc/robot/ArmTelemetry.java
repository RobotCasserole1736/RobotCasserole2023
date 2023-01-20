package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Arm.ArmEndEffectorPos;

public class ArmTelemetry {

    /* Singleton infratructure*/
    private static ArmTelemetry inst = null;
    public static synchronized ArmTelemetry getInstance() {
        if (inst == null)
            inst = new ArmTelemetry();
        return inst;
    }

    private final Mechanism2d m_mech2d = new Mechanism2d(90, 90);
    private final MechanismRoot2d midNodeHome = m_mech2d.getRoot("Mid Node", 27.83, 0);
    private final MechanismLigament2d MidNode = midNodeHome.append(new MechanismLigament2d("Mid Cone Node", 34, 90, 10, new Color8Bit(Color.kWhite)));
    private final MechanismRoot2d highNodeHome = m_mech2d.getRoot("High Node", 10.58, 0);
    private final MechanismLigament2d HighNode = highNodeHome.append(new MechanismLigament2d("High Cone Node", 46, 90, 10, new Color8Bit(Color.kWhite)));
    private final MechanismRoot2d gridHome = m_mech2d.getRoot("Grid Home", 49.75, 0);
    private final MechanismLigament2d GridNode = gridHome.append(new MechanismLigament2d("Grid Wall", 49.75, 180, 50, new Color8Bit(Color.kWhite)));
    private final MechanismRoot2d dsHome = m_mech2d.getRoot("Double Substation Home", 49.75, 37);
    private final MechanismLigament2d DSRamp = dsHome.append(new MechanismLigament2d("Double Substation Ramp", 13.75, 180, 10, new Color8Bit(Color.kWhite)));
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 65, 21.75);

    
    private final MechanismLigament2d m_arm_upper =
        m_armPivot.append(
              new MechanismLigament2d(
                "Arm Upper",
                27, 
                -90, 
                10, 
                new Color8Bit(Color.kSkyBlue)));
    private final MechanismLigament2d m_arm_tower =
        m_armPivot.append(new MechanismLigament2d("ArmTower", 18, -90, 10, new Color8Bit(Color.kSilver)));
  
    private final MechanismLigament2d m_aframe_1 =
        m_armPivot.append(new MechanismLigament2d("aframe1", 24, -50, 10, new Color8Bit(Color.kSilver)));
    private final MechanismLigament2d m_bumper =
        gridHome.append(new MechanismLigament2d("Bumper", 30.5, 0, 60, new Color8Bit(Color.kRed)));
    private final MechanismLigament2d m_arm_lower =
        m_arm_upper.append(
            new MechanismLigament2d(
                "Arm Lower",
                28.5,
                Units.radiansToDegrees(0.0),
                10,
                new Color8Bit(Color.kBlue)));

    private ArmTelemetry(){
        // Put Mechanism 2d to SmartDashboard
        SmartDashboard.putData("Arm", m_mech2d);
    }

    //TODO - add more ligaments for desired/measured visualization
    public void setActual(double upperAngleDeg, double lowerAngleDeg){
        m_arm_upper.setAngle(upperAngleDeg);
        m_arm_lower.setAngle(lowerAngleDeg);
    }
    public void setDesired(ArmEndEffectorPos des, double upperAngleDeg, double lowerAngleDeg){

    }
    public void setMeasured(double upperAngleDeg, double lowerAngleDeg){

    }
    
}
