
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;


import frc.Constants;
import frc.robot.Arm.ArmKinematics;
import frc.robot.Arm.ArmState;

class KinematicsTest {
  static final double DELTA = 1e-2; // acceptable deviation range

  @Test // marks this method as a test
  void test1() {

    var testArmState = new ArmState();
    testArmState.boomAngleDeg = 0; //assign angle inputs for testcase
    testArmState.stickAngleDeg = 0;

    //Run test inputs through the arm kinematics
    var result = ArmKinematics.forward(testArmState);

    assertEquals(result.x, (Constants.ARM_BOOM_LENGTH + Constants.ARM_STICK_LENGTH));
    assertEquals(result.y, (Constants.ARM_BOOM_MOUNT_HIEGHT));
  }

}
