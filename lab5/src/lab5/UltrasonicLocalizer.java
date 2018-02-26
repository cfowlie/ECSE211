package lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
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
	UltrasonicLocalizer() throws OdometerExceptions {
		
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

	/**
     * This is the falling edge method of USLocalization
     * This method finds the angles by looking for the distance to cross over 
     * the set distance from a larger value to a smaller value
     */
    void fallingEdge2() {
        
        // Variables
        double alpha;
        double beta;
        double deltaTheta;
        double average;
        double actualTheta;
        double currentTheta;
        double zeroDegrees;
        
        // Rotate clockwise until no wall is seen
        leftMotor.setSpeed(DriveManager.ROTATE_SPEED); 
        rightMotor.setSpeed(DriveManager.ROTATE_SPEED);
        while(sensorManager.getDistance() < DriveManager.NO_WALL_DIST) {
            leftMotor.forward();
            rightMotor.backward();
        }
        // Now rotate until the set distance has been reached
        while(sensorManager.getDistance() > this.d) {
            leftMotor.forward();
            rightMotor.backward();
        }  
        // once the set distance has been reached, stop rotating and save the angle
        Sound.beep();
        leftMotor.stop(true);
        rightMotor.stop(true);
        alpha = sensorManager.getOdometer().getXYT()[2];
        
        // Now rotate again until no wall is seen, but counterclockwise
        leftMotor.setSpeed(DriveManager.ROTATE_SPEED); 
        rightMotor.setSpeed(DriveManager.ROTATE_SPEED);
        while(sensorManager.getDistance() < DriveManager.NO_WALL_DIST) {
            leftMotor.backward();
            rightMotor.forward();
        }
        // Rotate the robot counterclockwise until the sensor has reached the set distance from the wall in the opposite direction
        while(sensorManager.getDistance() > this.d) {
            leftMotor.backward();
            rightMotor.forward();
        } 
        // once the set distance has been reached, stop rotating and save the angle
        Sound.beep();
        leftMotor.stop(true);
        rightMotor.stop(true);
        beta = sensorManager.getOdometer().getXYT()[2]; 
        
        // Now use alpha and beta to calculate deltaTheta
        average = Math.toDegrees((alpha + beta)/2);
        if(alpha < beta) {
            deltaTheta = 45 - average;
        } else {
            deltaTheta = 225 - average;
        }
        currentTheta = Math.toDegrees(sensorManager.getOdometer().getXYT()[2]);
        actualTheta = currentTheta + deltaTheta; // Correct theta by adding deltaTheta to the current heading
        zeroDegrees = 180 - actualTheta;         // Find the zero point based off our angle calculations
        // Rotate to 0 degrees
        leftMotor.setSpeed(DriveManager.ROTATE_SPEED);
        rightMotor.setSpeed(DriveManager.ROTATE_SPEED);
        leftMotor.rotate(DriveManager.convertAngle(zeroDegrees), true);
        rightMotor.rotate(-DriveManager.convertAngle(zeroDegrees), false);  
        
        // Initialize theta to zero degrees
        sensorManager.getOdometer().setTheta(0);
        
    }
	
}
