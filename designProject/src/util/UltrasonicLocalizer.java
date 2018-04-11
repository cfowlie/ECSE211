package util;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import main.DriveManager;
import main.SensorManager;
import odometer.OdometerExceptions;
import ultrasonic.UltrasonicPoller;

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

	/*
	 * Localize using falling edges
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

		driveManager.turnBy(zeroDegrees+90);

		// Set Odometer value and stop
		driveManager.stopAll();
		sensorManager.getOdometer().setTheta(0);
		
		return;
	}
	
}
