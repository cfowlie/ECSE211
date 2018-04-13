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
 * This class contains the methods that navigate the robot to the tunnel or
 * bridge and then traverse the tunnel or bridge.
 * 
 * @author Lucas Bluethner
 * @author David Castonguay
 * @version Final 1.0.5
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
	 * The following method brings the robot to the front of the tunnel, performs a
	 * light localization routine to make sure its perfectly in line, then travels
	 * through the tunnel.
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
		driveManager.forwardBy(-0.5 * DriveManager.TILE_SIZE - DriveManager.LIGHT_RADIUS);
		driveManager.turnBy(-90);
		driveManager.lineLocWait();

		// traverse through the tunnel
		tunnelSeq();

	}

	/**
	 * The following method brings the robot to the front of the bridge, performs a
	 * light localization routine to make sure its perfectly in line, expands its
	 * track by transforming, then travels over the bridge.
	 * 
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	public void traverseBridge() throws OdometerExceptions, InterruptedException {

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(1000);
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
		driveManager.forwardBy(-0.5 * DriveManager.TILE_SIZE - DriveManager.LIGHT_RADIUS);
		driveManager.turnBy(-90);
		driveManager.transform();
		driveManager.forwardBy(DriveManager.TILE_SIZE);
		driveManager.lineLocWait();

		// traverse the bridge
		bridgeSeq();

	}

	/**
	 * After the bridge and tunnel have both been traversed, this is the method that
	 * is called to bring the robot back to its starting corner.
	 * 
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	public void travelToStartCorner() throws OdometerExceptions, InterruptedException {
		driveManager.travelToGrid(driveManager.startCornerLoc()[0], driveManager.startCornerLoc()[1]);
		driveManager.stopAll();
	}

	/**
	 * Once lined up with the tunnel, this method is called to drive through the
	 * tunnel
	 * 
	 * @throws OdometerExceptions
	 */
	public void tunnelSeq() throws OdometerExceptions {
		driveManager.forwardBy(4.5 * DriveManager.TILE_SIZE - DriveManager.LIGHT_RADIUS);
	}

	/**
	 * Once lined up with the bridge, this method is called to drive over the
	 * bridge, the robot then contracts its track using the transform() method once
	 * it is over the bridge.
	 * 
	 * @throws OdometerExceptions
	 */
	public void bridgeSeq() throws OdometerExceptions {
		driveManager.forwardBy(3.5 * DriveManager.TILE_SIZE - DriveManager.LIGHT_RADIUS);
		driveManager.transform();
		driveManager.forwardBy(1 * DriveManager.TILE_SIZE);
	}

}