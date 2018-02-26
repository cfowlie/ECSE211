package lab5;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import odometer.Odometer;
import odometer.OdometerExceptions;
import ultrasonic.UltrasonicPoller;

public class UltrasonicLocalizer {

	double fallingEdge;
	double estimate;

	int d;
	int k;

	int slowSpeed = 65;

	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3UltrasonicSensor ultrasonicSensor;
	UltrasonicPoller usPoller;

	/*
	 * Ultrasonic Localizer Constructor
	 * 
	 * @param leftMotor
	 * 
	 * @param rightMotor
	 */
	UltrasonicLocalizer() throws OdometerExceptions {
		
		// Shared Maangers
		DriveManager driveManager = DriveManager.getInstance();
		SensorManager sensorManager = SensorManager.getInstance();
		
		// Motors
		this.leftMotor = driveManager.getLeftMotor();
		this.rightMotor = driveManager.getRightMotor();
		
		// Sensors
		this.usPoller = sensorManager.getUsPoller();

		// Distance and Noise Margin
		d = 30;
		k = 6;
	}

	/*
	 * Localize using falling edges
	 */
	public void fallingEdge() throws InterruptedException {

		Thread.sleep(1000);

		leftMotor.setSpeed(slowSpeed);
		rightMotor.setSpeed(slowSpeed);

		if (usPoller.getDistance() < d + k) { // Currently facing wall
			do { // Turn right
				leftMotor.forward();
				rightMotor.backward();
				Thread.sleep(1000);
			} while (usPoller.getDistance() < (d + k));
		}

		// Check for first falling edge
		while (usPoller.getDistance() > (d + k)) { // Turn right until first wall found
			leftMotor.forward();
			rightMotor.backward();
		}

		// Record Position of wall
		leftMotor.stop(true);
		rightMotor.stop(false);
		Thread.sleep(1000);
		double t1 = Odometer.position[2];

		// Check for second falling edge
		do {
			leftMotor.backward();
			rightMotor.forward();
			Thread.sleep(1000);
		} while (usPoller.getDistance() > d - k);

		// Record Position of wall
		leftMotor.stop(true);
		rightMotor.stop(false);
		Thread.sleep(1000);
		double t2 = Odometer.position[2];

		// Localization Calculation
		double dt = t2 - t1;

		if (dt < -180) {
			dt += 360;
		} else if (dt >= 180) {
			dt -= 360;
		}

		// Rotate to 0 degrees
		double angle = dt / 2;

		Odometer.odo.setTheta(-angle);

		leftMotor.rotate(DriveManager.convertAngle(angle), true);
		rightMotor.rotate(-DriveManager.convertAngle(angle), false);

		// Set Odometer value and stop
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		return;
	}

}
