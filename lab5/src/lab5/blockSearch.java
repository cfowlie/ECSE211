package lab5;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import odometer.Odometer;
import odometer.OdometerExceptions;

public class BlockSearch {

	private static int i = 0;

	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;

	DriveManager driveManager = DriveManager.getInstance();
	SensorManager sensorManager = SensorManager.getInstance();

	boolean blockFound = false;

	BlockSearch() throws OdometerExceptions {

		// Motors
		this.leftMotor = driveManager.getLeftMotor();
		this.rightMotor = driveManager.getRightMotor();
	}

	public void search() throws InterruptedException, OdometerExceptions {

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		Thread.sleep(1000);

		leftMotor.setSpeed(DriveManager.ROTATE_SPEED);
		rightMotor.setSpeed(DriveManager.ROTATE_SPEED);

		double dx = Lab5.URx - Lab5.LLx;

		// Y-dist in cm
		double DDy = (Lab5.URy - Lab5.LLy) * DriveManager.TILE_SIZE;

		// X-dist in cm
		double DDx = (Lab5.URx - Lab5.LLx) * DriveManager.TILE_SIZE;

		double xORy = 0;

		do {
			for (; i <= 3; i++) {

				if (i == 0 || i == 2) {
					xORy = DDy;
				} else {
					xORy = DDx;
				}

				double dist = 0;

				double currentX = sensorManager.getOdometer().getXYT()[0];
				double currentY = sensorManager.getOdometer().getXYT()[1];

				while (dist < xORy) {
					leftMotor.forward();
					rightMotor.forward();
					if (sensorManager.getSideDistance() <= 35) {
						driveManager.stopAll();
						Sound.beep();
						blockDetected();
						if (blockFound) {
							driveManager.forwardBy(xORy - dist - DriveManager.ULTRA_OFFSET);
							return;
						} else {
							driveManager.forwardBy(15); // Blocks must be 10cm apart, stops from seeing same block twice
						}
					}

					// Get distance already traveled
					double deltaX = sensorManager.getOdometer().getXYT()[0] - currentX;
					double deltaY = sensorManager.getOdometer().getXYT()[1] - currentY;
					dist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
				}
				driveManager.turnBy(90);
			}

		} while (!blockFound);

	}

	void blockDetected() throws InterruptedException, OdometerExceptions {

		// Correct for forward sensor offset
		driveManager.forwardBy(DriveManager.ULTRA_OFFSET);

		// Stop
		driveManager.stopAll();

		// Wait to stop properly
		Thread.sleep(500);

		// Turn torwards the block
		driveManager.turnBy(90);

		double temp1x = Odometer.position[0];
		double temp1y = Odometer.position[1];

		// Get distance to block
		while (sensorManager.getDistance() > 4) {
			leftMotor.forward();
			rightMotor.forward();
		}

		// Stop
		driveManager.stopAll();

		// Check Color
		if (sensorManager.getColor() == Lab5.TB) {
			// Play tone and exit
			Sound.playTone(1000, 200);
			blockFound = true;
		} else {
			Sound.playTone(300, 200);
			Thread.sleep(200);
			Sound.playTone(300, 200);
		}

		double temp2x = Odometer.position[0];
		double temp2y = Odometer.position[1];

		double distx = temp2x - temp1x;
		double disty = temp2y - temp1y;

		double stage_distance = Math.sqrt(Math.pow(distx, 2) + Math.pow(disty, 2));

		driveManager.forwardBy(-DriveManager.ULTRA_OFFSET);

		// Turn around
		driveManager.turnBy(-180);

		// Go distance
		driveManager.forwardBy((int) stage_distance - DriveManager.ULTRA_OFFSET);

		driveManager.turnBy(90);

	}

}
