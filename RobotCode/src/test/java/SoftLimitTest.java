
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Arm.ArmSoftLimits;


class SoftLimitTest {

  @Test 
  void testArmSoftLimitBasic() {
    ArmSoftLimits asl = new ArmSoftLimits();
    double[] listForXLimits = {1,2,};
    double[] listForYLimits = {1.1, 1,}; // change these to change the points that dictate the boundry. 
    // RESTRICTION LINES CAN NOT BE VERTICLE OR HORIZONTLE
     boolean testValueForSoftLimits = asl.robotSoftLimits(1.2,1.5,1.2,1.3,listForYLimits,listForXLimits);
System.out.println(testValueForSoftLimits);
  }

 

}
