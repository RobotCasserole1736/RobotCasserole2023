package frc.robot.Arm;

import edu.wpi.first.wpilibj.Timer;
import frc.Constants;
import frc.hardwareWrappers.AbsoluteEncoder.WrapperedAbsoluteEncoder;
import frc.hardwareWrappers.AbsoluteEncoder.WrapperedAbsoluteEncoder.AbsoluteEncType;
import frc.hardwareWrappers.MotorCtrl.WrapperedCANMotorCtrl;
import frc.lib.Signal.Annotations.Signal;
import frc.robot.ArmTelemetry;

public class ArmControl {

    WrapperedCANMotorCtrl upperMotor;
    WrapperedCANMotorCtrl lowerMotor;

    WrapperedAbsoluteEncoder upperEncoder;
    WrapperedAbsoluteEncoder lowerEncoder;



    //TODO - define many possible ArmPositions for all the locations we want to place or pickup from
    // These are totally temporary and for test only
    ArmEndEffectorPos start = new ArmEndEffectorPos(0.5, 0.5);
    ArmEndEffectorPos end = new ArmEndEffectorPos(1.5, 0.75);

    double startTime = Timer.getFPGATimestamp();

    ArmPath path;

    @Signal
    double xPos = 0;

    @Signal
    double yPos = 0;

    public ArmControl(){

        upperMotor = new WrapperedCANMotorCtrl("Arm Upper Motor", Constants.ARM_UPPER_MOTOR_CANID, WrapperedCANMotorCtrl.CANMotorCtrlType.SPARK_MAX);
        lowerMotor = new WrapperedCANMotorCtrl("Arm Lower Motor", Constants.ARM_LOWER_MOTOR_CANID, WrapperedCANMotorCtrl.CANMotorCtrlType.SPARK_MAX);
        upperEncoder = new WrapperedAbsoluteEncoder(AbsoluteEncType.SRXEncoder, "Arm Upper Enc", Constants.ARM_UPPER_ENC_IDX, 0.0); //TODO offsets
        lowerEncoder = new WrapperedAbsoluteEncoder(AbsoluteEncType.SRXEncoder, "Arm Lower Enc", Constants.ARM_LOWER_ENC_IDX, 0.0); //TODO offsets
      
        reset();
    }

    //TODO this is jsut for ease of testing, might need to be removed or refactored later
    public void reset(){
        //TODO - don't just regenerate path on reset, do it every time the commanded position changes
        //TODO - use arm's actual position and velocity for the start of the trajectory, not some assumed location
        //TODO - figure out if the path involves transitioning the arm between concave and convex, and slowly transition those angles over time?
        //TODO - maybe here or in a kiunematics class (and related to above) - figure out how we'll handle nice motor ramp rates when the start/end position is the same, but 
        // the joints still need to rotate to transition concave up to concave down. Or something like that
        path = new ArmPath(start, end, 2, 4);
        startTime = Timer.getFPGATimestamp();
    }


    public void update(){
        var desPos = path.sample(Timer.getFPGATimestamp() - startTime);

        //TODO - send des position through kinematics to determine desired joint angles and velocities



        ArmTelemetry.getInstance().setDesired(desPos, 0, 0); //TODO need to put actual desired angles in there
    }
    
}
