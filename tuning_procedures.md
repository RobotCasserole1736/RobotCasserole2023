Software Tuning Schedule

# Initial Checkout

Ensure the robot is up on blocks

Deploy latest software onto roboRIO.

Stay in Disabled at all times

Go through all faults - ensure none are unexpected

Confirm loop timing is running at about 20ms (plot signal in dashboard). 

Confirm data logs are being saved and can be retrieved and cleared via the website

Update any motor controller or device firmware as needed

Confirm at least some data is coming from each connected device

Confirm CAN bus load is under 75% 

Confirm all voltage rails on the RIO are good

# Swerve Encoder Offsets

This is a one-time procedure to determine and account for the offsets induced by our swerve drive module magnet and encoder mounting. It's expected to change every time we re-assemble a swerve module.

Ensure the robot is up on blocks, all of this is done with the robot disabled.

By hand, mechanically align all azimuth modules to be facing forward. Use the large carpenters square to ensure they are all aligned with (square to) the frame rails.

Read off the actual (measured) swerve module orientation for each module.

Plug in the measurement as the offset to constants.java . Re-load code.

Confirm all four modules now read zero degrees when mechanically aligned forward

Confirm 90/180/270 degree measurements as the wheels are rotated by hand counter-clockwise.

Visually confirm the driver dashboard indicators match the physical rotation of each module.

# Arm Constant Updates

in constants.java:

Confirm ARM_BOOM_GEAR_RATIO and ARM_STICK_GEAR_RATIO match the final chosen gear ratios on the boom/stick.

Update ARM_BOOM_MOUNT_HIEGHT to the actual height from the floor up to the center of the hex shaft that is the boom's pivot point.

Update ARM_BOOM_LENGTH to be the final center-to-center distance from the boom hex shaft to the stick hex shaft.

Update ARM_STICK_LENGTH to be the final distance from the center of the stick hex shaft out to the center of where the gamepieces are gripped in the intake/claw.

# Arm Encoder Offsets

## Boom

Ensure the robot is level on the ground. All this is done while disabled. All movement should be done by manually adjusting the arm.

Lift the boom to be level with the ground ( confirm with a level). This should be zero degrees. It does not matter where the stick is at.

Plot the boom measured boom angle in the stripcharts page of the robot website.

Set ARM_BOOM_ENCODER_MOUNT_OFFSET_RAD in constants.java to whatever the boom angle is reading.

Re-load code with updated number.

Confirm boom angle measures zero when straight out, positive when up, negative when down. 

## Stick

Same as boom, but adjust the variable ARM_STICK_ENCODER_MOUNT_OFFSET_RAD, and ensure the zero degrees is when the stick is in line with the boom, positive is angled upward while the boom is at 0 degrees, and negative is when angled downward while the boom is at 0 degrees.

## Together

Verify in Glass that the visualization of the arm moves in sync with how the arm is manually positioned.

Manually move the arm into each position for picking up or placing each cube. Record the x and y positions, and update the values in ArmNamedPosition.java to match.

# Swerve Drive Tuning - Manual

## Azimuth

The motors and gear ratios are unchanged since last year. Leave them as-is.

## Wheel

Prepare for tuning by looking through the wpilib flywheel tuning simulation:

https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html

Set all wheel gains to zero:

Drivetrain Module Wheel kP
Drivetrain Module Wheel kI
Drivetrain Module Wheel kD
Drivetrain Module Wheel kV
Drivetrain Module Wheel kS

Enable in test mode. All four modules should turn their azimuth to align to be straight forward.

Manually push the robot forward and backward. Ensure, in Glass, that the measured position of the robot changes correctly. Measure the actual distance moved with a tape measure matches the change in actual position of the odometry. Adjust drivetrain gear ratios or wheel radiuses in Constants.java if not.

Plot desired adn actual motor speed for each wheel.

Enable the fg_dt_wheel function generator to generate a square wave, with 0.5 m/sec amplititude.

Increase kS to be as large as possible, but without any actual motion occuring.

Increase kV to try to get the actual and desired speeds to match in steady state.

Increase kP to make the velocity converge faster, just about up until oscillation starts to occur.

Increase kD to help reduce oscillation. Back kP off a bit more if needed.

kI should not generally be used or needed in this case, because we have kV. If we have steady state error, adjust kV to get rid of it.

Try with faster speeds (increase amplititude of the function generator).

Switch the function generator to a sine wave and make sure the drivetrain can track the sine wave relatively well.

# Arm Position Control Tuning

Review and understand the WPILib example on tuning a vertical arm. This should largely apply to both segments of our arm. https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html

Additionally, all this can be practiced in simulation.

Have multiple laptops up and running. One should have the Calibrations page from the robot website up. The other should be plotting stripcharts. A third should have the code up and be ready to punch in calibration values or make code changes. A fourth should have the driver station and controls. 

Start with all gains at zero.

Arm Stick kF
Arm Stick kG
Arm Stick kS
Arm Stick kP
Arm Stick kI
Arm Stick kD
Arm Boom kF
Arm Boom kG
Arm Boom kS
Arm Boom kP
Arm Boom kI
Arm Boom kD

In all cases, plot the actual position and the desired position on the same stripchart. Plot the voltage applied to the motor on a second chart. 

## Function Generator Usage

While enabled in TEST mode in the driver station (not teleop or auto), function generators are available to generate test waveforms of desired values for each closed-loop controlled mechanism.

fg_<name>_type - type of waveform to generate. 0 = dissabled, 1 = square, 2 = sawtooth, 3 = sine, 4 = constant value (offset)
fg_<name>_freq - rate of change of the output in Hz.
fg_<name>_amp  - Amplititude of the output.
fg_<name>_offset - mean value to output. Output will oscillate around this value with _amp amplititude at a frequency of _freq.

Generally, for these angle mechanisms, start with an offset of 0 degrees, and amplitiude of a few degrees (say, 15), and a low frequency (say, 0.1 Hz, or 10 seconds between changes). Increase the frequency and amplititude to test different ranges of motion.

Square waves are good to start for tuning. Use sine waves to validate the ability to track a moving target.

Always be ready to disable the robot if the tuning values cause the arm to move unexpectedly. Switching the function generator type back to zero should also stop motion.

## Stick

Update the kG to get to the point where gravity is largely offset. When enabled, the stick should be poked with a stick, and largely hold its position.

Ensure that positive motor voltage is causing the measured angle to increase. Invert the motor in code if not.

Note kG may be close to zero on our arm, due to brake mode on the motor controller and the high gear ratio.

Start increasing kS to be as high as you can make it, without causing any actual motion. If the stick starts to move, decrease kS slightly.

Then, start increasing kV slightly. See how close you can make the actual/desired match with only feed-forward.

Then, start increasing kP. Allow the control to be aggressive, but not oscillate.

Try to avoid kI, but it also may be necessary once a gamepiece is picked up and held (variable load).

## Boom

Same procedure as stick, just use the boom constants and reset the stick constants back to zero.

## Together

Start with the arm out in space. Enable in teleop (non-test) mode.

Use the operator joysticks to manually move the desired position around slowly. Ensure the arm end effector tracks the desired position (and the actual/desired angles for boom and stick track well)