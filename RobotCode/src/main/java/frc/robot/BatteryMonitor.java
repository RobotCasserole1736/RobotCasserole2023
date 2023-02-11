package frc.robot;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.lib.Faults.Fault;
import frc.lib.Signal.Annotations.Signal;
import frc.Constants;

public class BatteryMonitor {
	private PowerDistribution pd;

	@Signal(units="V")
	double rioVoltage;
	@Signal(units="V")
	double batteryVoltage;
	@Signal(units="A")
	double batteryAmps;
	@Signal(units="Status")
	boolean rioBrownOutStatus;
	@Signal(units="V")
	double busRail3v3;
	@Signal(units="V")
	double busRail5v;
	@Signal(units="V")
	double busRail6v;

	@Signal(units="A")
	double cubeBlower;

	@Signal(units="A")
	double boomCurrent;
	@Signal(units="A")
	double stickCurrent;

	@Signal(units="A")
	double intakeCurrentLeft;
	@Signal(units="A")
	double intakeCurrentRight;

	@Signal(units="A")
	double wheelCurrentFL;
	@Signal(units="A")
	double azmthCurrentFL;
	@Signal(units="A")
	double wheelCurrentFR;
	@Signal(units="A")
	double azmthCurrentFR;
	@Signal(units="A")
	double wheelCurrentBL;
	@Signal(units="A")
	double azmthCurrentBL;
	@Signal(units="A")
	double wheelCurrentBR;
	@Signal(units="A")
	double azmthCurrentBR;

	@Signal(units="count")
	double canRXErrors;
	@Signal(units="count")
	double canTXErrors;
	@Signal(units="pct")
	double canBusLoad;

	Fault brownoutFault = new Fault("Battery Monitor", "Brownout");
	Fault rio5vRailFault = new Fault("Battery Monitor", "RIO 5V Rail Faulted");
	Fault rio6vRailFault = new Fault("Battery Monitor", "RIO 6V Rail Faulted");
	Fault rio3v3RailFault = new Fault("Battery Monitor", "RIO 3.3V Rail Faulted");

	final int UPDATE_RATE_MS = 100;

	LinearFilter canBusLoadFilter = LinearFilter.movingAverage(40);

	private static BatteryMonitor moniter = null;
	public static synchronized BatteryMonitor getInstance() {
		if(moniter == null)
			moniter = new BatteryMonitor();
		return moniter;
	}

	private BatteryMonitor() {
		pd = new PowerDistribution(0,ModuleType.kCTRE);
		
		// Kick off monitor in brand new thread.
	    // Thanks to Team 254 for an example of how to do this!
	    Thread monitorThread = new Thread(new Runnable() {
	        @Override
	        public void run() {
	            try {
	            	while(!Thread.currentThread().isInterrupted()){
	            		update();
	            		Thread.sleep(100);
	            	}
	            } catch (Exception e) {
	                e.printStackTrace();
	            }

	        }
		});

	    //Set up thread properties and start it off
	    monitorThread.setName("BatteryMonitor");
	    monitorThread.setPriority(Thread.MIN_PRIORITY);
	    monitorThread.start();
		
	}

	

	private void update (){ 
		batteryVoltage = pd.getVoltage();
		batteryAmps = pd.getTotalCurrent();
		rioVoltage = RobotController.getBatteryVoltage();

		rioBrownOutStatus = RobotController.isBrownedOut();
		busRail3v3 = RobotController.getVoltage3V3();
		busRail5v = RobotController.getVoltage5V();
		busRail6v = RobotController.getVoltage6V();

		CANStatus tmp = RobotController.getCANStatus();
		canRXErrors = tmp.receiveErrorCount;
		canTXErrors = tmp.transmitErrorCount;
		canBusLoad = canBusLoadFilter.calculate(100.0 * tmp.percentBusUtilization);

		brownoutFault.set(rioBrownOutStatus);
		rio3v3RailFault.set(!RobotController.getEnabled3V3() && !rioBrownOutStatus);
		rio5vRailFault.set(!RobotController.getEnabled5V() && !rioBrownOutStatus);
		rio6vRailFault.set(!RobotController.getEnabled6V() && !rioBrownOutStatus);

		cubeBlower = pd.getCurrent(frc.Constants.CUBE_BLOWER_CURRENT_CHANNEL);
		boomCurrent = pd.getCurrent(frc.Constants.BOOM_CURRENT_CHANNEL);
		stickCurrent = pd.getCurrent(frc.Constants.STICK_CURRENT_CHANNEL);
		intakeCurrentLeft = pd.getCurrent(frc.Constants.STICK_CURRENT_CHANNEL);
		intakeCurrentRight = pd. getCurrent(frc.Constants.RIGHT_INTAKE_CURRENT_CHANNEL);
		wheelCurrentFL = pd.getCurrent(frc.Constants.FL_WHEEL_CURRENT_CHANNEL);
		azmthCurrentFL = pd.getCurrent(frc.Constants.FL_AZMTH_CURRENT_CHANNEL);
		wheelCurrentFR = pd.getCurrent(frc.Constants.FR_WHEEL_CURRENT_CHANNEL);
		azmthCurrentFR = pd.getCurrent(frc.Constants.FR_AZMTH_CURRENT_CHANNEL);
		wheelCurrentBL = pd.getCurrent(frc.Constants.BL_WHEEL_CURRENT_CHANNEL);
		azmthCurrentBL = pd.getCurrent(frc.Constants.BL_AZMTH_CURRENT_CHANNEL);
		wheelCurrentBR = pd.getCurrent(frc.Constants.BR_WHEEL_CURRENT_CHANNEL);
		azmthCurrentBR = pd.getCurrent(frc.Constants.BR_AZMTH_CURRENT_CHANNEL);
	}
}