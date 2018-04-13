package util;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import main.DriveManager;
import main.SensorManager;
import odometer.OdometerExceptions;
import ultrasonic.UltrasonicPoller;

/**
 * This is the class that contains the ultrasonic localization routine used to
 * orient the robot in its starting corner.
 * 
 * @author Lucas Bluethner
 * @author Connor Fowlie
 *
 */
public class UltrasonicLocalizer {

	double fallingEdge;
	double estimate;

	int d;
	int k;

	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3UltrasonicSensor ultrasonicSensor;
	UltrasonicPoller usPoller;

	// Shared Maangers
	DriveManager driveManager = DriveManager.getInstance();
	SensorManager sensorManager = SensorManager.getInstance();

	/*
	 * Ultrasonic Localizer Constructor
	 * 
	 * @param leftMotor
	 * 
	 * @param rightMotor
	 */
	public UltrasonicLocalizer() throws OdometerExceptions {

		// Motors
		this.leftMotor = driveManager.getLeftMotor();
		this.rightMotor = driveManager.getRightMotor();

		// Sensors
		this.usPoller = sensorManager.getUsPoller();

		// Distance and Noise Margin
		d = 30;
		k = 4;
	}

	/**
	 * This is the method that orients the robot in its starting corner using a
	 * routine called the fallingEdge method. The robot rotates towards the wall
	 * until it reaches a set distance, records the angle, then rotates in the
	 * opposite direction until it reaches the set distance again and records the
	 * angle. The two recorded angles are then used to calculate the angle that must
	 * be added to robots current heading so the robot knows its actual angle
	 * relative to the grid map.
	 * 
	 * @throws InterruptedException
	 * @throws OdometerExceptions
	 */
	public void fallingEdge() throws InterruptedException, OdometerExceptions {

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(500);
		}

		Thread.sleep(200);

		leftMotor.setSpeed(DriveManager.ROTATE_SPEED);
		rightMotor.setSpeed(DriveManager.ROTATE_SPEED);

		if (usPoller.getDistance() < d + k) { // Currently facing wall
			do { // Turn right
				leftMotor.forward();
				rightMotor.backward();
				Thread.sleep(200);
			} while (usPoller.getDistance() < (d + k));
		}

		// Check for first falling edge
		while (usPoller.getDistance() > (d - k)) { // Turn right until first wall found
			leftMotor.forward();
			rightMotor.backward();
		}

		// Record Position of wall
		leftMotor.stop(true);
		rightMotor.stop(false);
		Thread.sleep(200);
		double

		alpha = sensorManager.getOdometer().position[2];

		if (usPoller.getDistance() < d + k) { // Currently facing wall
			do { // Turn right
				leftMotor.backward();
				rightMotor.forward();
				Thread.sleep(200);
			} while (usPoller.getDistance() < (d + k));
		}

		// Check for second falling edge
		do {
			leftMotor.backward();
			rightMotor.forward();
			Thread.sleep(200);
		} while (usPoller.getDistance() > d - k);

		// Record Position of wall
		leftMotor.stop(true);
		rightMotor.stop(false);
		Thread.sleep(200);
		double beta = sensorManager.getOdometer().position[2];

		// Now use alpha and beta to calculate deltaTheta
		double average = (alpha + beta) / 2;
		double deltaTheta;
		if (alpha < beta) {
			deltaTheta = 45 - average;
		} else {
			deltaTheta = 225 - average;
		}
		double currentTheta = sensorManager.getOdometer().getXYT()[2];
		double actualTheta = currentTheta + deltaTheta; // Correct theta by adding deltaTheta to the current heading
		double zeroDegrees = 180 - actualTheta; // Find the zero point based off our angle calculations

		driveManager.turnBy(zeroDegrees + 90);

		// Set Odometer value and stop
		driveManager.stopAll();
		sensorManager.getOdometer().setTheta(0);

		return;
	}

}
