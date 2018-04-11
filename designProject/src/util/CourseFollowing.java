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
 * Course following class is meant mainly for Beta Demo as a way to hardcode the
 * process.
 * 
 * Author: David Cast
 */

public class CourseFollowing {

	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3LargeRegulatedMotor leftUpMotor;
	EV3LargeRegulatedMotor rightUpMotor;

	DriveManager driveManager = DriveManager.getInstance();
	SensorManager sensorManager = SensorManager.getInstance();

	// travel the Euclidian distance of two times the light radius

	public CourseFollowing() throws OdometerExceptions {

		// Motors
		this.leftMotor = driveManager.getLeftMotor();
		this.rightMotor = driveManager.getRightMotor();
		this.leftUpMotor = driveManager.getLeftUpMotor();
		this.rightUpMotor = driveManager.getRightUpMotor();

		// Sensors

	}

	/**
	 * The following method brings the robot to the front of the tunnel, then
	 * travels through it
	 * 
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	public void traverseTunnel() throws OdometerExceptions, InterruptedException {

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		// prevents the upper motors from rotating (locks them in place)
		leftUpMotor.stop();
		rightUpMotor.stop();

		// wait two seconds to know its the light starting
		Thread.sleep(500);

		// If the green team starts in corner 2 or 3, it must travel to the upper part
		// of the tunnel
		// If the red team starts in corner 0 or 1, it must travel travel to the upper
		// part of the tunnel
		if ((!DriveManager.TEAM && (DriveManager.T12_SC == 2 || DriveManager.T12_SC == 3))
				|| (DriveManager.TEAM && (DriveManager.T12_SC == 0 || DriveManager.T12_SC == 1))) {
			driveManager.travelToGrid(DriveManager.TN_URx - 0.5, DriveManager.TN_URy + 1.5);
			driveManager.turnTo(180);
		} else {
			// otherwise, the robot travels to the lower part of the bridge
			driveManager.travelToGrid(DriveManager.TN_LLx + 0.5, DriveManager.TN_LLy - 1.5);
			driveManager.turnTo(0);
		}

		// straighten out the robot before going through the tunnel
		driveManager.turnBy(90);
		driveManager.lineLocWait();
		driveManager.forwardBy(-0.5*DriveManager.TILE_SIZE);
		driveManager.turnBy(-90);
		driveManager.lineLocWait();
		
		// traverse through the tunnel
		tunnelSeq();

	}

	/**
	 * The following method brings the robot to the front of the bridge, then
	 * travels over it
	 * 
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	public void traverseBridge() throws OdometerExceptions, InterruptedException {

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		// prevents the upper motors from rotating (locks them in place)
		leftUpMotor.stop();
		rightUpMotor.stop();

		// wait two seconds to know its the light starting
		Thread.sleep(500);

		// If the green team starts in corner 2 or 3, it must travel to the lower part
		// of the bridge
		// If the red team starts in corner 0 or 1, it must travel travel to the lower
		// part of the bridge
		if ((!DriveManager.TEAM && (DriveManager.T12_SC == 2 || DriveManager.T12_SC == 3))
				|| (DriveManager.TEAM && (DriveManager.T12_SC == 0 || DriveManager.T12_SC == 1))) {
			driveManager.travelToGrid(DriveManager.BR_LLx + 0.5, DriveManager.BR_LLy - 2.5);
			driveManager.turnTo(0);
		} else {
			// otherwise, the robot travels to the upper part of the bridge
			driveManager.travelToGrid(DriveManager.BR_URx - 0.5, DriveManager.BR_URy + 2.5);
			driveManager.turnTo(180);
		}

		// straightens out the robot before going over the bridge
		driveManager.turnBy(90);
		driveManager.lineLocWait();
		driveManager.forwardBy(-0.5*DriveManager.TILE_SIZE);
		driveManager.turnBy(-90);
		driveManager.transform();
		driveManager.forwardBy(DriveManager.TILE_SIZE);
		driveManager.lineLocWait();
		
		// traverse the bridge
		bridgeSeq();

	}

	/**
	 * After the bridge and tunnel have both been traversed, this method brings the
	 * robot back to its starting location
	 * 
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	public void travelToStartCorner() throws OdometerExceptions, InterruptedException {
		driveManager.travelToGrid(driveManager.startCornerLoc()[0], driveManager.startCornerLoc()[1]);
		driveManager.stopAll();
	}

	public void tunnelSeq() throws OdometerExceptions {
		driveManager.forwardBy(4.5 * DriveManager.TILE_SIZE - DriveManager.LIGHT_RADIUS);
	}

	public void bridgeSeq() throws OdometerExceptions {
		driveManager.forwardBy(3.5 * DriveManager.TILE_SIZE - DriveManager.LIGHT_RADIUS);
		driveManager.transform();
		driveManager.forwardBy(1 * DriveManager.TILE_SIZE);
	}

}
