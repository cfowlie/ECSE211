package util;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import main.DriveManager;
import main.SensorManager;
import odometer.Odometer;
import odometer.OdometerExceptions;
import ultrasonic.UltrasonicPoller;

public class BlockSearch {

	private static int i = 0;

	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3UltrasonicSensor ultrasonicSensor;
	UltrasonicPoller usPoller;
	
	
	DriveManager driveManager = DriveManager.getInstance();
	SensorManager sensorManager = SensorManager.getInstance();
	
	private static double instUSDist;
	
	private static double afterBlockDist;

	boolean blockFound = false;

	public BlockSearch() throws OdometerExceptions {

		// Motors
		this.leftMotor = driveManager.getLeftMotor();
		this.rightMotor = driveManager.getRightMotor();
		
		this.usPoller = sensorManager.getUsPoller();
		
	}
	
	
	
	public void search() throws InterruptedException, OdometerExceptions {

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		Thread.sleep(1000);
	
		driveManager.setRotSpd();  //prefered slower speed for block searching so it misses nothing.

		double dx = DriveManager.SR_URx - DriveManager.SR_LLx;
		double dy = DriveManager.SR_URy - DriveManager.SR_LLy;

		// Y-dist in cm
		double DDy = (DriveManager.SR_URy - DriveManager.SR_LLy) * DriveManager.TILE_SIZE;

		// X-dist in cm
		double DDx = (DriveManager.SR_URx - DriveManager.SR_LLx) * DriveManager.TILE_SIZE;

		double xORy = 0;
		
		driveManager.turnTo(0);

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
					do  {
					leftMotor.forward();
					rightMotor.forward();
					} while(usPoller.getDistance() > 55);
					
						driveManager.stopAll();
						instUSDist = usPoller.getDistance();
						Sound.beep();
						blockDetected();
						if (blockFound) {
							driveManager.travelTo(DriveManager.SG_URx,DriveManager.SG_URy,true);
							return;
						} else {
							driveManager.forwardBy(15); // Blocks must be 10cm apart, stops from seeing same block twice
						}
					
				
					// Get distance already traveled
						driveManager.travelTo(DriveManager.SG_URx,DriveManager.SG_URy, true);
				}
				driveManager.turnBy(90);
			}

		} while (!blockFound);
		
		

	}

	void blockDetected() throws InterruptedException, OdometerExceptions {

		afterBlockDist = 0.22*instUSDist+DriveManager.ULTRA_OFFSET;
		
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
		while (sensorManager.getEuclidColor() < 0.04) {
			leftMotor.forward();
			rightMotor.forward();
		}

		// Stop
		driveManager.stopAll();
		
		

		// Check Color
		if (sensorManager.getColor() == DriveManager.T12_FLAG) {
			// Play tone and exit
			Sound.playTone(1000, 200);
			blockFound = true;
		} else {
			Sound.playTone(300, 200);
			Thread.sleep(200);
			Sound.playTone(300, 200);
			Thread.sleep(200);
			Sound.playTone(300, 200);
		}

		double temp2x = Odometer.position[0];
		double temp2y = Odometer.position[1];

		double distx = temp2x - temp1x;
		double disty = temp2y - temp1y;

		double stage_distance = Math.sqrt(Math.pow(distx, 2) + Math.pow(disty, 2));

		driveManager.forwardBy(-(int) stage_distance);

		driveManager.turnBy(-90);

	}
	
	
	

}
