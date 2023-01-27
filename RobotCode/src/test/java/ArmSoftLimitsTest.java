
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import frc.Constants;
import frc.robot.Arm.ArmEndEffectorPos;
import frc.robot.Arm.ArmKinematics;
import frc.robot.Arm.ArmSoftLimits;
import frc.robot.Arm.ArmState;

class KinematicsTest {
  //Floating point numbers can have a bit of error, that's ok in this case.
  static final double DELTA = 1e-10; // acceptable deviation range

  @Test 
  void testInsideZone1() {

    ArmSoftLimits asl = new ArmSoftLimits();

    ArmEndEffectorPos testPos = new ArmEndEffectorPos(5, 5, true);
    var retVal = asl.applyLimit(testPos);

    assertEquals(testPos, retVal);


  }

}
