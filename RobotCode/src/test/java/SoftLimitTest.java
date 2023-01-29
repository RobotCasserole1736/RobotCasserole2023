
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;


import frc.robot.Arm.ArmSoftLimits;


class SoftLimitTest {

  @Test 
  void testArmSoftLimitBasic() {
    ArmSoftLimits asl = new ArmSoftLimits();
    var resolts = asl.intersect(2, 2, 4, 2);
    assertEquals(2, resolts);
  }

 

}
