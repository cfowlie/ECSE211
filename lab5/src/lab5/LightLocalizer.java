package lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import odometer.Odometer;

public class LightLocalizer {

	private static final int ROTATE_SPEED = 60;
	private static final int FWD_SPEED = 100;
	private static final double LIGHT_RADIUS = 13.4;

	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3ColorSensor colorSensor;
	
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3ColorSensor colorSensor) {
		// Motors
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		// Sensors
		this.colorSensor = colorSensor;
	}
	
	/*
	 * Light localizer find origin method
	 * 
	 */
	public void findOrigin() throws InterruptedException {

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		
		// wait two seconds to know its the light starting
		Thread.sleep(2000);

		// set speed to 100
		leftMotor.setSpeed(FWD_SPEED);
		rightMotor.setSpeed(FWD_SPEED);

		// wait until black line at y=0
		while (colorSensor.getColorID() < 10) {
			leftMotor.forward();
			rightMotor.forward();
		}
		// set the y coordinate to y=light radius
		Odometer.odo.setY(LIGHT_RADIUS);

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// turn 90 degrees to go towards the x=0 line
		leftMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), true);
		rightMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 90), false);

		leftMotor.setSpeed(FWD_SPEED);
		rightMotor.setSpeed(FWD_SPEED);

		// wait until the x=0 line
		while (colorSensor.getColorID() < 10) {
			leftMotor.forward();
			rightMotor.forward();

		}
		// set the x coordinate to x= light radius
		Odometer.odo.setX(LIGHT_RADIUS);

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// turn back towards the (0,0) coordinate
		leftMotor.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 45), true);
		rightMotor.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 45), false);

		// travel the euclidian distance of two timea lthe light radius
		double LR2 = Math.sqrt(2 * Math.pow(LIGHT_RADIUS, 2));

		leftMotor.setSpeed(FWD_SPEED);
		rightMotor.setSpeed(FWD_SPEED);

		leftMotor.rotate(-convertDistance(Lab5.WHEEL_RAD, LR2), true);
		rightMotor.rotate(-convertDistance(Lab5.WHEEL_RAD, LR2), false);

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// wait until line y=0 to become straight.
		while (colorSensor.getColorID() < 10) {
			leftMotor.backward();
			rightMotor.forward();
		}

		// done
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
