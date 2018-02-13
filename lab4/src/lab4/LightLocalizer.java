package lab4;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import odometer.Odometer;

public class LightLocalizer {

	public static final EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));

	private static final int ROTATE_SPEED = 60;
	private static final int FWD_SPEED = 100;
	private static final double LIGHT_RADIUS = 6.80;

	public static void findOrigin(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) throws InterruptedException {

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		
		Thread.sleep(2000);
		

		leftMotor.setSpeed(FWD_SPEED);
		rightMotor.setSpeed(FWD_SPEED);

		while (colorSensor.getColorID() < 10){
			leftMotor.forward();
			rightMotor.forward();
		} 
		
		Odometer.odo.setY(LIGHT_RADIUS);
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 90), true);
		rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 90), false);
		
		leftMotor.setSpeed(FWD_SPEED);
		rightMotor.setSpeed(FWD_SPEED);

		while(colorSensor.getColorID() < 10) {
			leftMotor.forward();
			rightMotor.forward();
			
		} 
		
			Odometer.odo.setX(LIGHT_RADIUS);
			
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);

		leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 45), true);
		rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 45), false);

		
//		double LR2 = Math.sqrt(Math.pow(Odometer.position[0], 2)+Math.pow(Odometer.position[1], 2));
		
		double LR2 = Math.sqrt(2 * Math.pow(LIGHT_RADIUS, 2));

		leftMotor.setSpeed(FWD_SPEED);
		rightMotor.setSpeed(FWD_SPEED);
		
		
		leftMotor.rotate(-convertDistance(Lab4.WHEEL_RAD, LR2),true);
		rightMotor.rotate(-convertDistance(Lab4.WHEEL_RAD, LR2),false);
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		while(colorSensor.getColorID() < 10) {
			leftMotor.backward();
			rightMotor.forward();
		}
		
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
