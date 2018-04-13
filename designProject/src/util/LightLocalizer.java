package util;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import main.DriveManager;
import main.SensorManager;
import odometer.Odometer;
import odometer.OdometerExceptions;

/**
 * This class contains the method used to localize the robot to the starting
 * coordinates after the ultrasonic localization routine is done.
 * 
 * @author Connor Fowlie
 * @author David Castonguay
 * @author Lucas Bluethner
 * @version Final 1.0.5
 *
 */
public class LightLocalizer {

	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3LargeRegulatedMotor leftUpMotor;
	EV3LargeRegulatedMotor rightUpMotor;
	EV3ColorSensor lightRSensor;
	EV3ColorSensor lightLSensor;

	DriveManager driveManager = DriveManager.getInstance();
	SensorManager sensorManager = SensorManager.getInstance();

	// travel the euclidian distance of two timea lthe light radius

	public LightLocalizer() throws OdometerExceptions {

		// Motors
		this.leftMotor = driveManager.getLeftMotor();
		this.rightMotor = driveManager.getRightMotor();
		this.leftUpMotor = driveManager.getLeftUpMotor();
		this.rightUpMotor = driveManager.getRightUpMotor();

	}

	/**
	 * This method uses two light sensors located just in front of the two front
	 * wheels to localize itself over top of the starting corner coordinates facing
	 * the proper direction, depending on which corner it is in.
	 * 
	 * @throws InterruptedException
	 * @throws OdometerExceptions
	 */
	public void findOrigin() throws InterruptedException, OdometerExceptions {

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(1000);
		}

		// prevents the upper motors (used for transformation) locked in place
		leftUpMotor.stop();
		rightUpMotor.stop();

		// wait two seconds to know its the light starting
		Thread.sleep(200);

		// performs the line waiting action to put the robot in a perfect perpendicular
		// line with the black lines
		driveManager.lineLocWait();

		// Move forward by the offset distance of the light sensors so the front wheels
		// are over the black line
		driveManager.forwardBy(-DriveManager.LIGHT_RADIUS);

		driveManager.setRotSpd();

		// turn 90 degrees to go towards the x=0 line
		driveManager.turnBy(90);

		// performs the line waiting action to put the robot in a perfect perpendicular
		// line with the black lines
		driveManager.lineLocWait();

		// Move forward by the offset distance of the light sensors so the front wheels
		// are over the black line
		driveManager.forwardBy(-DriveManager.LIGHT_RADIUS);

		driveManager.setRotSpd();

		// set (x, y, theta) to the starting corner coordinates
		sensorManager.getOdometer().setXYT(DriveManager.TILE_SIZE * driveManager.startCornerLoc()[0],
				DriveManager.TILE_SIZE * driveManager.startCornerLoc()[1], driveManager.startCornerLoc()[2]);

		driveManager.turnBy(-90);

		driveManager.forwardBy(DriveManager.LIGHT_RADIUS - 5);

		driveManager.lineLocWait();

		driveManager.forwardBy(-DriveManager.LIGHT_RADIUS);

		// done
		driveManager.stopAll();
	}

}
