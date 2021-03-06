package main;

import lejos.hardware.Button;
import wifi.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import odometer.Display;
import odometer.OdometerExceptions;

import util.*;

/**
 * Main class which contains the main method, the first method run when the program is started.
 * 
 * @author Connor Fowlie
 * @author David Castonguay
 * @author Lucas Bluethner
 * @version Final 1.0.5
 * 
 */
public class Main {

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();

	/**
	 * This is the main method where instances and threads for DriveManager and
	 * SensorManager are created, Threads are created for the odometry and display,
	 * and the order of operations is decided depending on whether we are the Red
	 * Team or the Green Team.
	 * 
	 * @param args
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		// Init shared Managers
		final DriveManager driveManager = DriveManager.getInstance();
		final SensorManager sensorManager = SensorManager.getInstance();

		// LCD
		lcd.clear();

		// Odo Display
		Display odometryDisplay = new Display(lcd);
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		// Localizers
		final UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
		final LightLocalizer lightLocalizer = new LightLocalizer();

		// sections of the process
		final CourseFollowing courseFollowing = new CourseFollowing();
		final BlockSearch blockSearch = new BlockSearch();
		final Wifi wifi = new Wifi();

		// Setup Drive Thread
		driveManager.setDriveThread(new DriveThread() {

			@Override
			public void run() throws InterruptedException, OdometerExceptions {

				// transfers all the data from the wifi server into the robot
				wifi.transmit();

				// Ultrasonic localize
				ultrasonicLocalizer.fallingEdge();

				// Light localize
				lightLocalizer.findOrigin();

				/*
				 * if red team: traverse the bridge first, then search for target block then
				 * traverse the tunnel finally, go back to start corner
				 */
				if (DriveManager.TEAM) {
					courseFollowing.traverseBridge();
					blockSearch.search();
					driveManager.beep6();
					courseFollowing.traverseTunnel();
					courseFollowing.travelToStartCorner();
					/*
					 * if green team: traverse the tunnel first, then search for target block then
					 * traverse the bridge finally, go back to start corner
					 */
				} else {
					courseFollowing.traverseTunnel();
					blockSearch.search();
					driveManager.beep6();
					courseFollowing.traverseBridge();
					courseFollowing.travelToStartCorner();
				}

				completion();
			}

			@Override
			public void completion() throws OdometerExceptions {
				driveManager.stopAll();
			}
		});

		Thread.sleep(2500); // Wait to make sure all threads have initialized before starting drive code

		// Start Drive Thread asynchronously
		driveManager.start();

		Button.waitForAnyPress(); // Wait to exit program
		System.exit(0);
	}

}
