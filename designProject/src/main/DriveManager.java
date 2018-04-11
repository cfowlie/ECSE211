package main;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import odometer.OdometerData;
import odometer.OdometerExceptions;

interface DriveThread {

	/*
	 * Run robot code on async thread
	 */
	void run() throws InterruptedException, OdometerExceptions;

	/*
	 * Completion method for callback
	 */
	void completion() throws OdometerExceptions;
}

public class DriveManager {

	// Singleton Object
	private static DriveManager sharedManager = null;

	// Movement thread
	private static DriveThread driveThread = null;
	private static Thread thread;

	// Motor Objects
	private final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private final EV3LargeRegulatedMotor leftUpMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private final EV3LargeRegulatedMotor rightUpMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

	// Constants
	public static final double WHEEL_RAD = 2.0;
	public static final double TRACK_OPEN = 21.2;
	public static final double TRACK_CLOSED = 15.4;
	public static final int FWD_SPEED = 280;
	public static final int ROTATE_SPEED = 180;
	public static final int SL_ROTATE_SPEED = 90;
	public static final int ROTATE_UP_SPEED = 20;
	public static final double UP_ROTATION = 40;
	public static final double LIGHT_RADIUS = -5;
	public static final double LR2 = Math.sqrt(2 * Math.pow(DriveManager.LIGHT_RADIUS, 2));
	public static final double NO_WALL_DIST = 35;
	public static final double TILE_SIZE = 30.48;
	public static final double ULTRA_OFFSET = 5.0;

	public static int RedTeam;
	public static int RedCorner;
	public static int GreenTeam;
	public static int GreenCorner;

	public static int OG;
	public static int OR;

	public static int Red_LLx;
	public static int Red_LLy;
	public static int Red_URx;
	public static int Red_URy;

	public static int Green_LLx;
	public static int Green_LLy;
	public static int Green_URx;
	public static int Green_URy;

	public static int TN_LLx;
	public static int TN_LLy;
	public static int TN_URx;
	public static int TN_URy;

	public static int BR_LLx;
	public static int BR_LLy;
	public static int BR_URx;
	public static int BR_URy;

	public static int SR_LLx;
	public static int SR_LLy;
	public static int SR_URx;
	public static int SR_URy;

	public static int SG_LLx;
	public static int SG_LLy;
	public static int SG_URx;
	public static int SG_URy;

	public static boolean TEAM; // TRUE == red, FALSE == green

	/**
	 * 
	 * Values to be used as a replacement of red and green, instead we use
	 * "T12_FLAG"
	 * 
	 */

	public static int T12_FLAG;

	public static int T12_SC;

	public static int T12_SLLx;
	public static int T12_SLLy;
	public static int T12_SURx;
	public static int T12_SURy;

	/**
	 * TODO: All of these values need to be assigned and attributed in the
	 * CourseFollowing class The way we assign them will be with the wifi program
	 * which will have a "get" method to call the values
	 * 
	 */

	public static int trackState = 0;

	/*
	 * Setup Drivemanager threads
	 */
	private DriveManager() throws OdometerExceptions {
		setThread((new Thread() {
			public void run() {
				try {
					DriveManager.driveThread.run();
				} catch (InterruptedException | OdometerExceptions e) {
					// TODO Auto-generated catch block
				}
			}
		}));
	}

	/*
	 * Starts the drive thread
	 */
	public void start() {
		this.getThread().start();
	}

	/*
	 * Gets the drive manager instance. Use this to access drive manager
	 */
	public static DriveManager getInstance() throws OdometerExceptions {
		if (sharedManager == null) {
			sharedManager = new DriveManager();
		}
		return sharedManager;
	}

	/**
	 * This method causes the robot to turn (on point) by amount theta
	 * 
	 * @param theta
	 * @throws OdometerExceptions
	 */
	public void turnBy(double theta) throws OdometerExceptions {
		setRotSpd();
		getLeftMotor().rotate(DriveManager.convertAngle(theta), true);
		getRightMotor().rotate(-DriveManager.convertAngle(theta), false);
		return;
	}

	/**
	 * This method causes the robot to travel forward by a distance (cm)
	 * 
	 * @param dist
	 * @throws OdometerExceptions
	 */
	public void forwardBy(double dist) throws OdometerExceptions {
		setDriveSpd();
		getLeftMotor().rotate(DriveManager.convertDistance(dist), true);
		getRightMotor().rotate(DriveManager.convertDistance(dist), false);
		return;
	}

	public void forwardBySlow(double dist) throws OdometerExceptions {
		setRotSpd();
		getLeftMotor().rotate(DriveManager.convertDistance(dist), true);
		getRightMotor().rotate(DriveManager.convertDistance(dist), false);
		return;
	}

	/*
	 * Gets the current track width because our robot has different track widths
	 * depending on stage state.
	 */
	public static double widthCheck() {
		if (trackState == 0) {
			return TRACK_CLOSED;
		} else {
			return TRACK_OPEN;
		}

	}

	/*
	 * Causes the robot to transform from its current state to the other
	 */
	public void transform() {
		if (trackState == 1) {
			leftUpMotor.setSpeed(ROTATE_UP_SPEED);
			rightUpMotor.setSpeed(ROTATE_UP_SPEED);

			leftUpMotor.rotate(-45, true);
			rightUpMotor.rotate(-45, true);

			trackState = 0;
		} else {
			leftUpMotor.setSpeed(ROTATE_UP_SPEED);
			rightUpMotor.setSpeed(ROTATE_UP_SPEED);

			leftUpMotor.rotate(40, true);
			rightUpMotor.rotate(40, true);

			trackState = 1;
		}
	}

	/**
	 * Returns the proper starting corner information.
	 * 
	 * @return
	 * 
	 * 
	 */

	public double[] startCornerLoc() {
		double[] loc = new double[5];
		if (T12_SC == 0) {
			loc[0] = 1;
			loc[1] = 1;
			loc[2] = 90;
			loc[3] = +.5;
			loc[4] = +.5;
			return loc;
		} else if (T12_SC == 1) {
			loc[0] = 11;
			loc[1] = 1;
			loc[2] = 0;
			loc[3] = -.5;
			loc[4] = +.5;
			return loc;
		} else if (T12_SC == 2) {
			loc[0] = 11;
			loc[1] = 11;
			loc[2] = 270;
			loc[3] = -.5;
			loc[4] = -.5;
			return loc;
		} else {
			loc[0] = 1;
			loc[1] = 11;
			loc[2] = 180;
			loc[3] = +.5;
			loc[4] = -.5;
			return loc;
		}
	}

	/**
	 * This method causes the robot to turn towards an explicit heading
	 * 
	 * @param heading
	 * @throws OdometerExceptions
	 */
	public void turnTo(double headingT) throws OdometerExceptions {

		SensorManager sensorManager = SensorManager.getInstance();

		double position[] = sensorManager.getOdometer().getXYT();

		double currentT = position[2];

		double theta = headingT - currentT; // Calculate the angle the robot has to actually turn

		// This makes sure the robot always turns the smaller angle
		if (theta < -180) {
			theta += 360;
		} else if (theta > 180) {
			theta -= 360;
		}
		turnBy(theta);

	}

	/**
	 * This method causes the robot to travel to the absolute field location (x,y),
	 * specified in tile points.
	 * 
	 * @param x
	 * @param y
	 * @throws OdometerExceptions
	 */
	public void travelTo(double x, double y, boolean avoid) throws OdometerExceptions, InterruptedException {

		SensorManager sensorManager = SensorManager.getInstance();
		DriveManager driveManager = DriveManager.getInstance();

		// Get the current x, y and theta positions from the odometer
		double position[] = sensorManager.getOdometer().getXYT();
		double currentX = position[0];
		double currentY = position[1];
		double currentT = position[2];

		double dX = (TILE_SIZE * x) - currentX; // Calculate the distance the robot has left to travel in the x
												// direction
		double dY = (TILE_SIZE * y) - currentY; // Calculate the distance the robot has left to travel in the y
												// direction

		double headingT = Math.toDegrees(Math.atan2(dX, dY)); // Calculate the angle the robot need to turn to and

		double theta = headingT - currentT; // Calculate the angle the robot has to actually turn

		// This makes sure the robot always turns the smaller angle
		if (theta < -180) {
			theta += 360;
		} else if (theta > 180) {
			theta -= 360;
		}
		// Turn by theta degrees so that the robot is facing the direction is needs to
		// go
		turnBy(theta);

		lineLocWait();

		double posAfter[] = sensorManager.getOdometer().getXYT();
		currentX = posAfter[0];
		currentY = posAfter[1];
		currentT = posAfter[2];

		dX = (TILE_SIZE * x) - currentX; // Calculate the distance the robot has left to travel in the x
											// direction
		dY = (TILE_SIZE * y) - currentY; // Calculate the distance the robot has left to travel in the y

		// Calculate the distance the robot must travel to get to the way point
		double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));

		// Start robot forward towards the way point
		forwardBy(distance);

	}

	/*
	 * Travels to an X and Y position by traveling X first and then Y, along grid
	 * lines
	 */
	public void travelToGrid(double x, double y) throws OdometerExceptions, InterruptedException {
		SensorManager sensorManager = SensorManager.getInstance();
		double curX, curY;

		// Get the current x and y position
		curY = sensorManager.getOdometer().getXYT()[1] / DriveManager.TILE_SIZE;
		curX = sensorManager.getOdometer().getXYT()[0] / DriveManager.TILE_SIZE;
		// Only travel to the given x position if it is further than some distance
		if (Math.abs(x - curX) > 0.25) {
			travelTo(x, curY, true);
		}

		// Get the current x and y position
		curX = sensorManager.getOdometer().getXYT()[0] / DriveManager.TILE_SIZE;
		curY = sensorManager.getOdometer().getXYT()[1] / DriveManager.TILE_SIZE;
		// Only travel to the given y position if it is further than some distance
		if (Math.abs(y - curY) > 0.25) {
			travelTo(curX, y, true);
		}
	}

	/**
	 * This method causes the robot to travel to the absolute field location (x,y),
	 * specified in tile points.
	 * 
	 * @param xDist
	 *            the distance to travel in the x direction
	 * @param y
	 * @throws OdometerExceptions
	 */

	/*
	 * Stops all motors
	 */
	public void stopAll() throws OdometerExceptions {
		getLeftMotor().stop(true);
		getRightMotor().stop(false);
	}

	/**
	 * This method returns true if another thread has called travelTo() or turnTo()
	 * and the method has yet to return; false otherwise
	 * 
	 * @return returns true if the robots motors are moving, false otherwise
	 */
	boolean isNavigating() {
		return getLeftMotor().isMoving() && getRightMotor().isMoving();
	}

	/*
	 * Convert distance in cm to motor rotation
	 */
	public static int convertDistance(double distance) {
		double radius = DriveManager.WHEEL_RAD;
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/*
	 * Convert and angle
	 */
	public static int convertAngle(double angle) {
		double width = widthCheck();
		return convertDistance(Math.PI * width * angle / 360.0);
	}

	/*
	 * Drive strait correction
	 */
	public void lineLocWait() throws InterruptedException, OdometerExceptions {

		SensorManager sensorManager = SensorManager.getInstance();
		setRotSpd();

		// wait until black line hits one of the two light sensors
		while (sensorManager.getLine() == 0) {
			leftMotor.forward();
			rightMotor.forward();
		}

		switch (sensorManager.getLine()) {
		case 1: // Left line
			stopAll();
			
			setSLRotSpd();
			while (sensorManager.getLine() != 2) {
				rightMotor.forward();
			}
			
			rightMotor.rotate(30);
			stopAll();
			break;
		case 2: // Right line
			stopAll();
			setSLRotSpd();
			while (sensorManager.getLine() != 1) {
				leftMotor.forward();
			}
			
			leftMotor.rotate(30);
			stopAll();
			break;
		case 3: // Both lines
			stopAll();
			break;
		}

		stopAll();

		if (OdometerData.roundToNearest90() == 0) {
			sensorManager.getOdometer().setTheta(0);
		} else if (OdometerData.roundToNearest90() == 1) {
			sensorManager.getOdometer().setTheta(90);
		} else if (OdometerData.roundToNearest90() == 2) {
			sensorManager.getOdometer().setTheta(180);
		} else {
			sensorManager.getOdometer().setTheta(270);
		}
		setRotSpd();

		Thread.sleep(50);
	}

	public void lineLocWaitDrive(double x, double y) throws InterruptedException, OdometerExceptions {

		SensorManager sensorManager = SensorManager.getInstance();
		setRotSpd();

		double position[] = sensorManager.getOdometer().getXYT();
		double currentX = position[0];
		double currentY = position[1];
		double currentT = position[2];

		double dX = 100;
		double dY = 100;

		while (Math.abs(dX) > 1.5*TILE_SIZE) {
			position = sensorManager.getOdometer().getXYT();
			currentX = position[0];
			currentY = position[1];
			currentT = position[2];

			dX = (TILE_SIZE * x) - currentX; // Calculate the distance the robot has left to travel in the x
												// direction
			dY = (TILE_SIZE * y) - currentY; // Calculate the distance the robot has left to travel in the y

			double headingT = Math.toDegrees(Math.atan2(dX, 0.001)); // Calculate the angle the robot need to turn to
																		// and

			double theta = headingT - currentT; // Calculate the angle the robot has to actually turn

			// This makes sure the robot always turns the smaller angle
			if (theta < -180) {
				theta += 360;
			} else if (theta > 180) {
				theta -= 360;
			}
			// Turn by theta degrees so that the robot is facing the direction is needs to
			// go
			turnBy(theta);
			// This makes sure the robot always turns the smaller angle

			// wait until black line hits one of the two light sensors
			while (sensorManager.getLine() == 0) {

				leftMotor.forward();
				rightMotor.forward();
			}

			switch (sensorManager.getLine()) {
			case 1: // Left line
				leftMotor.stop(true);
				setSLRotSpd();
				while (sensorManager.getLine() != 2) {
					rightMotor.forward();
					
				}
				rightMotor.rotate(15);
				break;
			case 2: // Right line
				rightMotor.stop(true);
				setSLRotSpd();
				while (sensorManager.getLine() != 1) {
					leftMotor.forward();
					
				}
				leftMotor.rotate(15);
				break;
			case 3: // Both lines
				break;
			}

			

			if (OdometerData.roundToNearest90() == 0) {
				sensorManager.getOdometer().setTheta(0);
			} else if (OdometerData.roundToNearest90() == 1) {
				sensorManager.getOdometer().setTheta(90);
			} else if (OdometerData.roundToNearest90() == 2) {
				sensorManager.getOdometer().setTheta(180);
			} else {
				sensorManager.getOdometer().setTheta(270);
			}
			setRotSpd();
			forwardBy(10);

		}

		travelTo(dX, currentY, true);

		position = sensorManager.getOdometer().getXYT();
		currentX = position[0];
		currentY = position[1];
		currentT = position[2];

		dX = (TILE_SIZE * x) - currentX; // Calculate the distance the robot has left to travel in the x
											// direction
		dY = (TILE_SIZE * y) - currentY; // Calculate the distance the robot has left to travel in the y
											// direction

		double headingT = Math.toDegrees(Math.atan2(0.001, dY)); // Calculate the angle the robot need to turn to and

		double theta = headingT - currentT; // Calculate the angle the robot has to actually turn

		// This makes sure the robot always turns the smaller angle
		if (theta < -180) {
			theta += 360;
		} else if (theta > 180) {
			theta -= 360;
		}
		// Turn by theta degrees so that the robot is facing the direction is needs to
		// go
		turnBy(theta);

		while (Math.abs(dY) > 1.5) {
			position = sensorManager.getOdometer().getXYT();
			currentX = position[0];
			currentY = position[1];
			currentT = position[2];

			dX = (TILE_SIZE * x) - currentX; // Calculate the distance the robot has left to travel in the x
												// direction
			dY = (TILE_SIZE * y) - currentY; // Calculate the distance the robot has left to travel in the y
												// direction

			// wait until black line hits one of the two light sensors
			while (sensorManager.getLine() == 0) {
				leftMotor.forward();
				rightMotor.forward();

			}

			switch (sensorManager.getLine()) {
			case 1: // Left line
				leftMotor.stop(true);
				setSLRotSpd();
				while (sensorManager.getLine() != 2) {
					rightMotor.forward();

				}
				break;
			case 2: // Right line
				rightMotor.stop(true);
				setSLRotSpd();
				while (sensorManager.getLine() != 1) {
					leftMotor.forward();

				}
			case 3: // Both lines
				break;
			}

			leftMotor.stop(true);
			rightMotor.stop(false);

			if (OdometerData.roundToNearest90() == 0) {
				sensorManager.getOdometer().setTheta(0);
			} else if (OdometerData.roundToNearest90() == 1) {
				sensorManager.getOdometer().setTheta(90);
			} else if (OdometerData.roundToNearest90() == 2) {
				sensorManager.getOdometer().setTheta(180);
			} else {
				sensorManager.getOdometer().setTheta(270);
			}
			setRotSpd();

			forwardBy(10);
		}

		travelTo(currentX, dY, true);

	}

	// MARK: Speed

	public void setRotSpd() { // setting wheels to a slower rotating speed
		leftMotor.setSpeed(DriveManager.ROTATE_SPEED);
		rightMotor.setSpeed(DriveManager.ROTATE_SPEED);
	}

	public void setSLRotSpd() { // setting wheels to a slower rotating speed
		leftMotor.setSpeed(DriveManager.SL_ROTATE_SPEED);
		rightMotor.setSpeed(DriveManager.SL_ROTATE_SPEED);
	}

	public void setDriveSpd() { // setting wheels to a faster forward speed
		leftMotor.setSpeed(DriveManager.FWD_SPEED);
		rightMotor.setSpeed(DriveManager.FWD_SPEED);
	}

	// MARK: Field Encapsulation

	/**
	 * @return the leftMotor
	 */
	public EV3LargeRegulatedMotor getLeftMotor() {
		return leftMotor;
	}

	/**
	 * @return the rightMotor
	 */
	public EV3LargeRegulatedMotor getRightMotor() {
		return rightMotor;
	}

	/**
	 * 
	 * @return the leftUpMotor
	 */
	public EV3LargeRegulatedMotor getLeftUpMotor() {
		return leftUpMotor;
	}

	/**
	 * @return the rightUpMotor
	 */
	public EV3LargeRegulatedMotor getRightUpMotor() {
		return rightUpMotor;
	}

	/**
	 * @return the driveThread
	 */
	public DriveThread getDriveThread() {
		return driveThread;
	}

	/**
	 * @param driveThread
	 *            the driveThread to set
	 */
	public void setDriveThread(DriveThread driveThread) {
		DriveManager.driveThread = driveThread;
	}

	/**
	 * @return the thread
	 */
	private Thread getThread() {
		return thread;
	}

	/**
	 * @param thread
	 *            the thread to set
	 */
	private static void setThread(Thread thread) {
		DriveManager.thread = thread;
	}

}
