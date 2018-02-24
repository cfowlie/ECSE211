package lab5;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import odometer.Odometer;
import ultrasonic.UltrasonicPoller;

public class UltrasonicLocalizer {

	double fallingEdge;
	double risingEdge;
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
	UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, UltrasonicPoller usPoller) {
		// Motors
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		// Sensors
		this.usPoller = usPoller;

		// Distance and Noise Margin
		d = 30;
		k = 6;
	}

	/*
	 * Localize using falling edges
	 */
	public void fallingEdge() throws InterruptedException {

		Thread.sleep(1000);

		Lab5.leftMotor.setSpeed(slowSpeed);
		Lab5.rightMotor.setSpeed(slowSpeed);

		if (usPoller.getDistance() < d + k) { // Currently facing wall
			do { // Turn right
				Lab5.leftMotor.forward();
				Lab5.rightMotor.backward();
				Thread.sleep(1000);
			} while (usPoller.getDistance() < d + k);
		}

		// Check for first falling edge
		while (usPoller.getDistance() > d + k) { // Turn right until first wall found
			Lab5.leftMotor.forward();
			Lab5.rightMotor.backward();
		}

		// Record Position of wall
		Lab5.leftMotor.stop(true);
		Lab5.rightMotor.stop(false);
		Thread.sleep(1000);
		double t1 = Odometer.position[2];

		// Check for second falling edge
		do {
			Lab5.leftMotor.backward();
			Lab5.rightMotor.forward();
			Thread.sleep(1000);
		} while (usPoller.getDistance() > d - k);

		// Record Position of wall
		Lab5.leftMotor.stop(true);
		Lab5.rightMotor.stop(false);
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

		Lab5.leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, angle), true);
		Lab5.rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, angle), false);

		// Set Odometer value and stop
		Lab5.leftMotor.stop(true);
		Lab5.rightMotor.stop(false);
		
		return;
	}

	/*
	 * Helper method to convert distances
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/*
	 * Helper method to convert angles
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
