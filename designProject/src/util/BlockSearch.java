package util;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import main.DriveManager;
import main.SensorManager;
import odometer.Odometer;
import odometer.OdometerExceptions;
import ultrasonic.UltrasonicPoller;
import wifi.Wifi;

/**
 * This class contains the two methods used to perform the block search routine.&nbsp;
 * One method traverses the perimeter of the search region searching for a, and the other method
 * goes to a block once one is detected and determines if it is the target block or not.
 * 
 * @author David Castonguay
 *
 */
public class BlockSearch {

	private static int i = 0;

	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3UltrasonicSensor ultrasonicSensor;
	UltrasonicPoller usPoller;

	DriveManager driveManager = DriveManager.getInstance();
	SensorManager sensorManager = SensorManager.getInstance();

	private static double instUSDist;

	private static final int BLOCK_DIST = 55;
	private static final double EUCLID_DIST = 0.05;

	private static double afterBlockDist;

	boolean blockFound = false;

	public BlockSearch() throws OdometerExceptions {

		// Motors
		this.leftMotor = driveManager.getLeftMotor();
		this.rightMotor = driveManager.getRightMotor();

		this.usPoller = sensorManager.getUsPoller();

	}

	/**
	 * This is the method that causes the robot to traverse around the perimeter of
	 * the search region, searching for blocks as it goes using the ultrasonic
	 * sensor.&nbsp;If a non-target block is found, the robot continues searching.&nbsp;Once
	 * the target block is found, it continues on its path back to its starting
	 * corner.
	 *
	 * @throws InterruptedException
	 * @throws OdometerExceptions
	 */
	public void search() throws InterruptedException, OdometerExceptions {

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(800);
		}

		Thread.sleep(1000);

		driveManager.setRotSpd(); // prefered slower speed for block searching so it misses nothing.

		double dx = DriveManager.T12_SURx - DriveManager.T12_SLLx;
		double dy = DriveManager.T12_SURy - DriveManager.T12_SLLy;

		// Y-dist in cm
		double DDy = dy * DriveManager.TILE_SIZE;

		// X-dist in cm
		double DDx = dx * DriveManager.TILE_SIZE;

		double xORy = 0;

		driveManager.travelTo(squareDist()[0][3], squareDist()[1][3], false);

		driveManager.turnTo(270);

		do {
			for (; i <= 3; i++) {

				driveManager.setRotSpd(); // preferred slower speed for block searching so it misses nothing.

				if (i == 0 || i == 2) {
					xORy = DDy;
				} else {
					xORy = DDx;
				}

				double dist = 0;

				double currentX = sensorManager.getOdometer().getXYT()[0];
				double currentY = sensorManager.getOdometer().getXYT()[1];

				while (dist < xORy) {

					// moves forward until it sees a block from the ultrasonic sensor
					do {
						leftMotor.forward();
						rightMotor.forward();
					} while (usPoller.getDistance() > BLOCK_DIST);
					driveManager.stopAll();
					// records the distance from which the block is located
					instUSDist = usPoller.getDistance();
					Sound.beep();
					blockDetected();
					if (blockFound) {
						if (DriveManager.T12_SC == 2 || DriveManager.T12_SC == 3) {
							if (DriveManager.TEAM) {
								driveManager.travelTo(DriveManager.TN_LLx, DriveManager.TN_LLy, true);
							} else {
								driveManager.travelTo(DriveManager.BR_LLx, DriveManager.BR_LLy, true);
							}
						} else {
							if (DriveManager.TEAM) {
								driveManager.travelTo(DriveManager.TN_URx, DriveManager.TN_URy, true);
							} else {
								driveManager.travelTo(DriveManager.BR_URx, DriveManager.BR_URy, true);

							}
						}
						return;
					} else {
						driveManager.forwardBy(15); // Blocks must be 10cm apart, stops from seeing same block twice
					}

					// Travel rest of the square
					driveManager.travelTo(squareDist()[0][i], squareDist()[1][i], false);
				}
				driveManager.turnBy(90);
			}

		} while (!blockFound);

	}

	/**
	 * The search method calls this method whenever a block is detected by the
	 * ultrasonic sensor.&nbsp;This method cause the robot to turn toward the block,
	 * travel to it and check its color.&nbsp;If it is the target block the robot beeps 3
	 * times, otherwise it beeps once.&nbsp;Next, the robot goes back to the perimeter
	 * and goes back to to where it was in the search method.
	 * 
	 * @throws InterruptedException
	 * @throws OdometerExceptions
	 */
	public void blockDetected() throws InterruptedException, OdometerExceptions {

		// drives a distance to get perfectly perpendicular to the block, works as a
		// linear function of distance.
		afterBlockDist = 0.3 * instUSDist + DriveManager.ULTRA_OFFSET;

		// Correct for forward sensor offset
		driveManager.forwardBy(afterBlockDist);

		// Stop
		driveManager.stopAll();

		// Wait to stop properly
		Thread.sleep(500);

		// Turn torwards the block
		driveManager.turnBy(90);

		double temp1x = Odometer.position[0];
		double temp1y = Odometer.position[1];

		// Get distance to block by using the light sensor's euclidian color as distance
		while (sensorManager.getEuclidColor() < EUCLID_DIST) {
			leftMotor.forward();
			rightMotor.forward();
		}

		// Stop
		driveManager.stopAll();

		// Check Color
		if (sensorManager.getColor() == DriveManager.T12_FLAG) {
			// Target block found, beep three times and exit
			for (int i = 0; i < 3; i++) {
				Sound.beep();
				Thread.sleep(200);
			}
			blockFound = true;
		} else {
			// Non-target block found, beep once
			Sound.playTone(300, 200);
		}

		double temp2x = Odometer.position[0];
		double temp2y = Odometer.position[1];

		double distx = temp2x - temp1x;
		double disty = temp2y - temp1y;

		double stage_distance = Math.sqrt(Math.pow(distx, 2) + Math.pow(disty, 2));

		// go back to perimeter.
		driveManager.forwardBy(-stage_distance);

		// repoint itself forward.
		driveManager.turnBy(-90);

	}

	/**
	 * Makes the robot go to the right corners during block search.&nbsp;Depending on if
	 * starting in corner 0, 1, 2 or 3, it will send the proper corners to go to
	 * after the end of one blocksearch loop.
	 * 
	 * @return corner to travel to
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	private int[][] squareDist() throws OdometerExceptions, InterruptedException {
		int[][] cornersXY = new int[2][4];
		int[] cornersX = new int[4];
		int[] cornersY = new int[4];

		if (DriveManager.T12_SC == 2 || DriveManager.T12_SC == 3) {
			cornersX[0] = DriveManager.T12_SURx;
			cornersY[0] = DriveManager.T12_SURy;
			cornersX[1] = DriveManager.T12_SURx;
			cornersY[1] = DriveManager.T12_SLLy;
			cornersX[2] = DriveManager.T12_SLLx;
			cornersY[2] = DriveManager.T12_SLLy;
			cornersX[3] = DriveManager.T12_SLLx;
			cornersY[3] = DriveManager.T12_SURy;
		} else {
			cornersX[0] = DriveManager.T12_SLLx;
			cornersY[0] = DriveManager.T12_SLLy;
			cornersX[1] = DriveManager.T12_SLLx;
			cornersY[1] = DriveManager.T12_SURy;
			cornersX[2] = DriveManager.T12_SURx;
			cornersY[2] = DriveManager.T12_SURy;
			cornersX[3] = DriveManager.T12_SURx;
			cornersY[3] = DriveManager.T12_SLLy;
		}
		cornersXY[0] = cornersX;
		cornersXY[1] = cornersY;
		return cornersXY;
	}

}
