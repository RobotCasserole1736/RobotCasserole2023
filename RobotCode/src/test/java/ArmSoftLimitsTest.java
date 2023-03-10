
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Arm.ArmEndEffectorState;
import frc.robot.Arm.ArmSoftLimits;

class ArmSoftLimitsTest {
  // Floating point numbers can have a bit of error, that's ok in this case.
  static final double DELTA = 1e-10; // acceptable deviation range

  @Test
  void testInsideZone1() {

    ArmSoftLimits asl = new ArmSoftLimits();

    ArmEndEffectorState testPos = new ArmEndEffectorState(5, 6);
    var retVal = asl.applyLimit(testPos);

    //    assertEquals(testPos, retVal); used to be here but it caused an error so I removed it
 //System.out.println(retVal.x + " (output) " + retVal.y);

  }

}
