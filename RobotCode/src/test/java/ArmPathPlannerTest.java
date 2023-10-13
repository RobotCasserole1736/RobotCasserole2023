
import org.junit.jupiter.api.Test;

import frc.robot.Arm.ArmEndEffectorState;
import frc.robot.Arm.ArmNamedPosition;
import frc.robot.Arm.Path.ArmPathFactory;

import static org.junit.jupiter.api.Assertions.*;



class ArmPathPlannerTest {
  
  @Test
  void testBasicToFrom() {

    for(double startX = -10; startX < 10; startX += 0.78){
      for(double startY = -10; startY < 10; startY += 0.78){
        for(var endPos : ArmNamedPosition.values()){

          var startState = new ArmEndEffectorState(startX, startY);
          var pathUnderTest = ArmPathFactory.build(startState, endPos);

          var pathDur = pathUnderTest.getDurationSec();

          assertTrue(pathDur > 0.0);
          assertEquals(startState.x, pathUnderTest.sample(-1.0).x);
          assertEquals(startState.y, pathUnderTest.sample(-1.0).y);
          assertEquals(startState.x, pathUnderTest.sample(0.0).x);
          assertEquals(startState.y, pathUnderTest.sample(0.0).y);
          assertEquals(endPos.get().x, pathUnderTest.sample(pathDur).x);
          assertEquals(endPos.get().y, pathUnderTest.sample(pathDur).y);
          assertEquals(endPos.get().x, pathUnderTest.sample(pathDur+0.25).x);
          assertEquals(endPos.get().y, pathUnderTest.sample(pathDur+0.25).y);        
        }
      }
    }
  }

  @Test
  void testSmallPaths() {

    for(var endPos : ArmNamedPosition.values()){
      for(double deltaX = -0.5; deltaX < 0.5; deltaX += 0.1){
        for(double deltaY = -0.5; deltaY < 0.5; deltaY += 0.1){

          var end = endPos.get();
          var startState = new ArmEndEffectorState(end.x + deltaX, end.y + deltaY);
          var pathUnderTest = ArmPathFactory.build(startState, endPos);

          var pathDur = pathUnderTest.getDurationSec();

          if(Math.abs(deltaX) >= 0.01 && Math.abs(deltaY) >= 0.01){
            assertTrue(pathDur > 0.0);
          }

          assertEquals(startState.x, pathUnderTest.sample(0.0).x);
          assertEquals(startState.y, pathUnderTest.sample(0.0).y);
          assertEquals(endPos.get().x, pathUnderTest.sample(pathDur).x);
          assertEquals(endPos.get().y, pathUnderTest.sample(pathDur).y);      
        }
      }
    }
  }
}
