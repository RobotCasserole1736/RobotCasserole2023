
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Arm.ArmSoftLimits;


class SoftLimitTest {

  @Test 
  void testArmSoftLimitBasic() {
    ArmSoftLimits asl = new ArmSoftLimits();
    double[] listForXLimits = {1,2};
    double[] listForYLimits = {1,2};// change these to change the points that dictate the boundry. 
    // RESTRICTION LINES CAN NOT BE VERTICLE OR HORIZONTLE
    asl.robotSoftLimits(0.1,0.3,0.1,0.1,listForXLimits,listForXLimits);

  }

 

}
