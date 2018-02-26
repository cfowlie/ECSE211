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
	
	public LightLocalizer() throws OdometerExceptions {
		
		DriveManager driveManager = DriveManager.getInstance();
		SensorManager sensorManager = SensorManager.getInstance();
		
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
		Odometer.odo.setY(DriveManager.LIGHT_RADIUS);

		leftMotor.setSpeed(DriveManager.ROTATE_SPEED);
		rightMotor.setSpeed(DriveManager.ROTATE_SPEED);

		// turn 90 degrees to go towards the x=0 line
		leftMotor.rotate(DriveManager.convertAngle(90), true);
		rightMotor.rotate(-DriveManager.convertAngle(90), false);

		leftMotor.setSpeed(DriveManager.FWD_SPEED);
		rightMotor.setSpeed(DriveManager.FWD_SPEED);

		// wait until the x=0 line
		while (colorSensor.getColorID() < 10) {
			leftMotor.forward();
			rightMotor.forward();

		}
		// set the x coordinate to x= light radius
		Odometer.odo.setX(DriveManager.LIGHT_RADIUS);

		leftMotor.setSpeed(DriveManager.ROTATE_SPEED);
		rightMotor.setSpeed(DriveManager.ROTATE_SPEED);

		// turn back towards the (0,0) coordinate
		leftMotor.rotate(-DriveManager.convertAngle(45), true);
		rightMotor.rotate(DriveManager.convertAngle(45), false);

		// travel the euclidian distance of two timea lthe light radius
		double LR2 = Math.sqrt(2 * Math.pow(DriveManager.LIGHT_RADIUS, 2));

		leftMotor.setSpeed(DriveManager.FWD_SPEED);
		rightMotor.setSpeed(DriveManager.FWD_SPEED);

		leftMotor.rotate(-DriveManager.convertDistance(LR2), true);
		rightMotor.rotate(-DriveManager.convertDistance(LR2), false);

		leftMotor.setSpeed(DriveManager.ROTATE_SPEED);
		rightMotor.setSpeed(DriveManager.ROTATE_SPEED);

		// wait until line y=0 to become straight.
		while (colorSensor.getColorID() < 10) {
			leftMotor.backward();
			rightMotor.forward();
		}
		
		Odometer.odo.setTheta(0);

		// done
		leftMotor.stop(true);
		rightMotor.stop(false);
	}
}
