package lab5;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import odometer.Odometer;
import ultrasonic.UltrasonicController;

public class UltrasonicLocalizer implements UltrasonicController {

	double fallingEdge;
	double risingEdge;
	double estimate;

	int d;
	int k;

	int slowSpeed = 65;

	int distance;

	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3UltrasonicSensor ultrasonicSensor;

	/*
	 * Ultrasonic Localizer Constructor
	 * 
	 * @param leftMotor
	 * 
	 * @param rightMotor
	 */
	UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		// Motors
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

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

		if (this.distance < d + k) { // Currently facing wall
			do { // Turn right
				Lab5.leftMotor.forward();
				Lab5.rightMotor.backward();
				Thread.sleep(1000);
			} while (this.distance < d + k);
		}

		// Check for first falling edge
		while (this.distance > d + k) { // Turn right until first wall found
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
		} while (this.distance > d - k);

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

		// Run Light Localization
		(new Thread() {
			public void run() {
				try {
					LightLocalizer.findOrigin(leftMotor, rightMotor);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}).start();

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

	// MARK: Ultrasonic Sensor

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
