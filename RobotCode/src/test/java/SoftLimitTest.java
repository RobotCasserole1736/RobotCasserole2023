
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Arm.ArmSoftLimits;


class SoftLimitTest {

  @Test 
  void testArmSoftLimitBasic() {
    ArmSoftLimits asl = new ArmSoftLimits();
    double[] listForXLimits = {1,4};
    double[] listForYLimits = {1,4};// change these to change the points that dictate the boundry. 
    // RESTRICTION LINES CAN NOT BE VERTICLE OR HORIZONTLE
    Boolean outputFromSoftLimits = asl.robotSoftLimits(0,3,listForXLimits,listForXLimits);

    System.out.println(outputFromSoftLimits);
  }

 

}
