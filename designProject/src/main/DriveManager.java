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
	public static final double WHEEL_RAD = 2.01;
	public static final double TRACK_OPEN = 21.2;
	public static final double TRACK_CLOSED = 15.4;
	public static final int ROTATE_SPEED = 140;
	public static final int SL_ROTATE_SPEED = 80;
	public static final int ROTATE_UP_SPEED = 20;
	public static final double UP_ROTATION = 40;
	public static final int FWD_SPEED = 200;
	public static final double LIGHT_RADIUS = 4.2;
	public static final double LR2 = Math.sqrt(2 * Math.pow(DriveManager.LIGHT_RADIUS, 2));
	public static final double NO_WALL_DIST = 35;
	public static final double TILE_SIZE = 30.48;

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
		getLeftMotor().setSpeed(ROTATE_SPEED);
		getRightMotor().setSpeed(ROTATE_SPEED);
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
		getLeftMotor().setSpeed(FWD_SPEED);
		getRightMotor().setSpeed(FWD_SPEED);
		getLeftMotor().rotate(DriveManager.convertDistance(dist), true);
		getRightMotor().rotate(DriveManager.convertDistance(dist), false);
		return;
	}

	/**
	 * This method causes the robot to travel forward by a set number of tiles
	 * 
	 * @param tile_amount
	 * @throws OdometerExceptions
	 */
	public void forwardByT(int tile_amount) throws OdometerExceptions {
		getLeftMotor().setSpeed(FWD_SPEED);
		getRightMotor().setSpeed(FWD_SPEED);

		for (int i = 0; i > tile_amount; i++) {

			getLeftMotor().rotate(DriveManager.convertDistance(tile_amount * TILE_SIZE), true);
			getRightMotor().rotate(DriveManager.convertDistance(tile_amount * TILE_SIZE), true);

		}
		return;
	}

	/*
	 * Gets the current track width
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
	 * 		should be 11 instead of 7**************************
	 */
	public double[] startCornerLoc() {
		double[] loc = new double[5];
		if (DriveManager.GreenCorner == 0) {
			loc[0] = 1;
			loc[1] = 1;
			loc[2] = 90;
			loc[3] = +.5;
			loc[4] = +.5;
			return loc;
		} else if (DriveManager.GreenCorner == 1) {
			loc[0] = 7;
			loc[1] = 1;
			loc[2] = 0;
			loc[3] = -.5;
			loc[4] = +.5;
			return loc;
		} else if (DriveManager.GreenCorner == 2) {
			loc[0] = 7;
			loc[1] = 7;
			loc[2] = 270;
			loc[3] = -.5;
			loc[4] = -.5;
			return loc;
		} else {
			loc[0] = 1;
			loc[1] = 7;
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
	public void travelTo(double x, double y, boolean avoid, boolean choice)
			throws OdometerExceptions, InterruptedException {

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

		if (avoid == true) {

			turnBy(theta);

			driveManager.lineLocWait();

			if (choice == true) {
				forwardBy(-DriveManager.LIGHT_RADIUS);

			}

			// Calculate the distance the robot must travel to get to the waypoint
			double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2)) - DriveManager.LIGHT_RADIUS;

			// Start robot forward towards the waypoint
			forwardBy((int) distance);

		}

		else {

			if (dY < 1) {
				if (dX > 0) {
					// if()
				}

			}

			forwardBy(dX);
			forwardBy(dY);
		}

		// Play sound when reaching location
		Sound.beep();
	}

	/*
	 * Travels to an X and Y position by traveling X first and then Y, along grid lines
	 */
	public void travelToGrid(double x, double y) throws OdometerExceptions, InterruptedException {
		SensorManager sensorManager = SensorManager.getInstance();

		double curX = sensorManager.getOdometer().getXYT()[0] / DriveManager.TILE_SIZE;
		double curY = sensorManager.getOdometer().getXYT()[1] / DriveManager.TILE_SIZE;
		if (Math.abs(y - curY) < 2) {

			travelTo(curX, y, true, false);
		}

		curY = sensorManager.getOdometer().getXYT()[1] / DriveManager.TILE_SIZE;
		curX = sensorManager.getOdometer().getXYT()[0] / DriveManager.TILE_SIZE;

		travelTo(x, curY, true, false);

	}
	
	/*
	 * Travels to an X and Y position by traveling X first and then Y, along grid lines
	 */
	public void travelToGridC(double x, double y) throws OdometerExceptions, InterruptedException {
		SensorManager sensorManager = SensorManager.getInstance();

		double curX = sensorManager.getOdometer().getXYT()[0] / DriveManager.TILE_SIZE;
		double curY = sensorManager.getOdometer().getXYT()[1] / DriveManager.TILE_SIZE;
		if (Math.abs(y - curY) < 2) {

			travelTo(curX, y, true, true);
		}

		curY = sensorManager.getOdometer().getXYT()[1] / DriveManager.TILE_SIZE;
		curX = sensorManager.getOdometer().getXYT()[0] / DriveManager.TILE_SIZE;

		travelTo(x, curY, true, true);

	}

	/**
	 * This method causes the robot to travel to the absolute field location (x,y),
	 * specified in tile points.
	 * 
	 * @param xDist the distance to travel in the x direction
	 * @param y
	 * @throws OdometerExceptions
	 */
	public void travelToDistance(double xDist, double yDist, boolean avoid) throws OdometerExceptions {

		SensorManager sensorManager = SensorManager.getInstance();

		// Get the current x, y and theta positions from the odometer
		double position[] = sensorManager.getOdometer().getXYT();
		double currentT = position[2];

		double headingT = Math.toDegrees(Math.atan2(xDist, yDist)); // Calculate the angle the robot need to turn to and

		double theta = headingT - currentT; // Calculate the angle the robot has to actually turn

		// This makes sure the robot always turns the smaller angle
		if (theta < -180) {
			theta += 360;
		} else if (theta > 180) {
			theta -= 360;
		}
		turnBy(theta);

		// Calculate the distance the robot must travel to get to the waypoint
		double distance = Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));

		// Start robot forward towards the waypoint
		forwardBy((int) distance);

		// Play sound when reaching location
		Sound.beep();
	}

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
		setDriveSpd();

		// wait until black line hits one of the two light sensors
		while (sensorManager.getLine() == 0) {
			leftMotor.forward();
			rightMotor.forward();
		}

		// if the right light sensor hit first, stop right motor and keep left running
		// until left light hits line
		if (sensorManager.getLine() == 2) {
			rightMotor.stop(true);
			setSLRotSpd();
			while (sensorManager.getLine() != 3) {
				leftMotor.forward();
			}
			leftMotor.rotate(20);

		}
		// if the left light sensor hit first, stop left motor and keep left running
		// until right light hits line
		else if (sensorManager.getLine() == 1) {
			leftMotor.stop(true);
			setSLRotSpd();
			while (sensorManager.getLine() != 3) {
				rightMotor.forward();
			}
			rightMotor.rotate(20);
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

		Thread.sleep(200);
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
