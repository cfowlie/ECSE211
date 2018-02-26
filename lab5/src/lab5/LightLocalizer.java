package lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import odometer.Odometer;
import odometer.OdometerExceptions;

public class LightLocalizer {

	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3ColorSensor colorSensor;
	
	DriveManager driveManager = DriveManager.getInstance();
	SensorManager sensorManager = SensorManager.getInstance();
	
	public LightLocalizer() throws OdometerExceptions {
		
		// Motors
		this.leftMotor = driveManager.getLeftMotor();
		this.rightMotor = driveManager.getRightMotor();
		
		// Sensors
		this.colorSensor = sensorManager.getLightSensor();
	}
	
	/*
	 * Light localizer find origin method
	 * 
	 */
	public void findOrigin()
			throws InterruptedException {

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		// wait two seconds to know its the light starting
		Thread.sleep(500);

		// set speed to 100
		leftMotor.setSpeed(DriveManager.FWD_SPEED);
		rightMotor.setSpeed(DriveManager.FWD_SPEED);

		// wait until black line at y=0
		while (colorSensor.getColorID() < 10) {
			leftMotor.forward();
			rightMotor.forward();
		}
		// set the y coordinate to y=light radius
		sensorManager.getOdometer().setY(DriveManager.LIGHT_RADIUS);

		// turn 90 degrees to go towards the x=0 line
		driveManager.turnBy(90);

		// wait until the x=0 line
		while (colorSensor.getColorID() < 10) {
			leftMotor.forward();
			rightMotor.forward();

		}
		// set the x coordinate to x= light radius
		sensorManager.getOdometer().setX(DriveManager.LIGHT_RADIUS);

		// turn back towards the (0,0) coordinate
		driveManager.turnBy(-45);

		// travel the euclidian distance of two timea lthe light radius
		double LR2 = Math.sqrt(2 * Math.pow(DriveManager.LIGHT_RADIUS, 2));

		driveManager.forwardBy(-DriveManager.convertDistance(LR2));

		leftMotor.setSpeed(DriveManager.ROTATE_SPEED);
		rightMotor.setSpeed(DriveManager.ROTATE_SPEED);

		// wait until line y=0 to become straight.
		while (colorSensor.getColorID() < 10) {
			driveManager.turnBy(2);
		}
		
		sensorManager.getOdometer().setTheta(0);

		// done
		driveManager.stopAll();
	}
}
