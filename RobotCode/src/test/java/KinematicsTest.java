
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;

import frc.Constants;
import frc.robot.Arm.ArmEndEffectorPos;
import frc.robot.Arm.ArmKinematics;
import frc.robot.Arm.ArmState;

class KinematicsTest {
  //Floating point numbers can have a bit of error, that's ok in this case.
  static final double DELTA = 1e-10; // acceptable deviation range

  @Test 
  void testStraightOut() {

    //assign angle inputs for testcase
    // straight out forward on the robot
    var testArmState = new ArmState();
    testArmState.boomAngleDeg = 0; 
    testArmState.stickAngleDeg = 0;

    //Run test inputs through the arm kinematics
    var result = ArmKinematics.forward(testArmState);

    assertEquals(result.x, (Constants.ARM_BOOM_LENGTH + Constants.ARM_STICK_LENGTH), DELTA);
    assertEquals(result.y, (Constants.ARM_BOOM_MOUNT_HIEGHT), DELTA);
  }

  @Test 
  void testStraightUp() {

    //straight up in the air
    var testArmState = new ArmState();
    testArmState.boomAngleDeg = 90; 
    testArmState.stickAngleDeg = 0;

    //Run test inputs through the arm kinematics
    var result = ArmKinematics.forward(testArmState);

    assertEquals(result.x, 0.0, DELTA);
    assertEquals(result.y, (Constants.ARM_BOOM_MOUNT_HIEGHT + Constants.ARM_BOOM_LENGTH + Constants.ARM_STICK_LENGTH), DELTA);
  }

  @Test 
  void testAngleUnbent() {

    //At an angle
    var testArmState = new ArmState();
    testArmState.boomAngleDeg = 45; 
    testArmState.stickAngleDeg = 0;

    //Run test inputs through the arm kinematics
    var result = ArmKinematics.forward(testArmState);
    var len = (Constants.ARM_BOOM_LENGTH + Constants.ARM_STICK_LENGTH) * Math.sqrt(2.0)/2.0;

    assertEquals(result.x, len, DELTA);
    assertEquals(result.y, (Constants.ARM_BOOM_MOUNT_HIEGHT + len), DELTA);
  }

  @Test 
  void testBent1() {

    //boom straight up, stick straight out
    var testArmState = new ArmState();
    testArmState.boomAngleDeg = 90; 
    testArmState.stickAngleDeg = -90;

    //Run test inputs through the arm kinematics
    var result = ArmKinematics.forward(testArmState);

    assertEquals(result.x, Constants.ARM_STICK_LENGTH, DELTA);
    assertEquals(result.y, (Constants.ARM_BOOM_MOUNT_HIEGHT + Constants.ARM_BOOM_LENGTH), DELTA);
    assertEquals(result.isReflex, true);
  }

  @Test 
  void testBent2() {

    //boom straight out, stick straight up
    var testArmState = new ArmState();
    testArmState.boomAngleDeg = 0; 
    testArmState.stickAngleDeg = 90;

    //Run test inputs through the arm kinematics
    var result = ArmKinematics.forward(testArmState);

    assertEquals(result.x, Constants.ARM_BOOM_LENGTH, DELTA);
    assertEquals(result.y, (Constants.ARM_BOOM_MOUNT_HIEGHT + Constants.ARM_STICK_LENGTH), DELTA);
    assertEquals(result.isReflex, false);
  }

  @Test 
  void testForwardInverse() {
    double boomAngleTestVals[] = {-90.0, 0.0, 1.0, -5.0, 10.0, -11.0, 14.5, 20.0, 90.0, -89.99, -1.0, 0.0, 0.1};
    double stickAngleTestVals[] = {0.1, -0.1, 1.0, -5.0, 90.0, 1.0, 10.0, -11.0, 14.5, 20.0, -90.0, 90.0, -89.99, 120, -140, 123.456, -124.256};

    for(double boomAngle : boomAngleTestVals){
      for(double stickAngle : stickAngleTestVals){
        var expected = new ArmState(boomAngle, stickAngle);
        var intRes = ArmKinematics.forward(expected);
        var actual = ArmKinematics.inverse(intRes);
        assertEquals(expected.boomAngleDeg, actual.boomAngleDeg, DELTA);
        assertEquals(expected.stickAngleDeg, actual.stickAngleDeg, DELTA);
      }
    }

  }

  @Test 
  void testInverseForward() {
    //Test values chosen to not request arm positions which are not physically possible
    double testVals[] = {0.3, 0.4, 0.5, 0.49999, -0.3, -0.45, -0.24, -0.3542 };
    boolean testReflexVals[] = {true, false};

    for(double x : testVals){
      for(double y : testVals){
        for(boolean reflex: testReflexVals){
          var y_adj = y + Constants.ARM_BOOM_MOUNT_HIEGHT; // adjust our refernce frame to be about the boom mount point
          var expected = new ArmEndEffectorPos(x, y_adj, reflex);
          var intRes = ArmKinematics.inverse(expected);
          var actual = ArmKinematics.forward(intRes);
          assertEquals(expected.x, actual.x, DELTA);
          assertEquals(expected.y, actual.y, DELTA);
          assertEquals(expected.isReflex, actual.isReflex);
        }
      }
    }

  }

}
