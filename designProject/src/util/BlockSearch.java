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
			motor.setAcceleration(800);
		}

		Thread.sleep(1000);
	
		driveManager.setRotSpd();  //prefered slower speed for block searching so it misses nothing.

		double dx = DriveManager.T12_SURx - DriveManager.T12_SLLx;
		double dy = DriveManager.T12_SURy - DriveManager.T12_SLLy;

		// Y-dist in cm
		double DDy = dy * DriveManager.TILE_SIZE;

		// X-dist in cm
		double DDx = dx * DriveManager.TILE_SIZE;

		double xORy = 0;
		
		
		driveManager.travelTo(squareDist()[0][0],squareDist()[1][0], false);
		
		driveManager.turnBy(90);

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
							if(DriveManager.T12_SC == 2 || DriveManager.T12_SC == 3) {
								
								if(DriveManager.TEAM) {
									driveManager.travelTo(DriveManager.TN_LLx,DriveManager.TN_LLy,true);}
								else {
									driveManager.travelTo(DriveManager.BR_LLx,DriveManager.BR_LLy,true);
							}
						}else{
							if(DriveManager.TEAM) {
								driveManager.travelTo(DriveManager.TN_URx,DriveManager.TN_URy,true);}
							else {
								driveManager.travelTo(DriveManager.BR_URx,DriveManager.BR_URy,true);	
							
							}
						}
							return;
						} else {
							driveManager.forwardBy(15); // Blocks must be 10cm apart, stops from seeing same block twice
						}
					
				
					// Travel rest of the square
						driveManager.travelTo(squareDist()[0][i],squareDist()[1][i], false);
				}
				driveManager.turnBy(90);
			}

		} while (!blockFound);
		
		

	}

	void blockDetected() throws InterruptedException, OdometerExceptions {

		afterBlockDist = 0.3*instUSDist+DriveManager.ULTRA_OFFSET;
		
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
	
	
	//makes the robot go to the right corners during block search.
	private int[][] squareDist() throws OdometerExceptions, InterruptedException {
		int[][] cornersXY = new int[2][4];
		int[] cornersX = new int[4];
		int[] cornersY = new int[4];
		
		if(DriveManager.T12_SC == 2 || DriveManager.T12_SC == 3) {
			
			cornersX[0]=DriveManager.T12_SURx; cornersY[0]=DriveManager.T12_SURy;
			cornersX[1]=DriveManager.T12_SURx; cornersY[1]=DriveManager.T12_SLLy;
			cornersX[2]=DriveManager.T12_SLLx; cornersY[2]=DriveManager.T12_SLLy;
			cornersX[3]=DriveManager.T12_SLLx; cornersY[3]=DriveManager.T12_SURy;
		
	}else{
		cornersX[0]=DriveManager.T12_SLLx; cornersY[0]=DriveManager.T12_SLLy;
		cornersX[1]=DriveManager.T12_SLLx; cornersY[1]=DriveManager.T12_SURy;
		cornersX[2]=DriveManager.T12_SURx; cornersY[2]=DriveManager.T12_SURy;
		cornersX[3]=DriveManager.T12_SURx; cornersY[3]=DriveManager.T12_SLLy;
	}
		cornersXY[0]=cornersX;
		cornersXY[1]=cornersY;
		return cornersXY;
	}
	

}
