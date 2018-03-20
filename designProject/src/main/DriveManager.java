package main;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
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
	private final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	// Constants
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 16.9;
	public static final int ROTATE_SPEED = 100;
	public static final int FWD_SPEED = 180;
	public static final double LIGHT_RADIUS = 14.5;
	public static final double NO_WALL_DIST = 35;
	public static final double TILE_SIZE = 31.0;
	public static final int ULTRA_OFFSET = 7;


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
	
	public void start() {
		this.getThread().start();	
	}

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
     * @param theta
	 * @throws OdometerExceptions 
     */
    public void forwardBy(double dist) throws OdometerExceptions {
    		getLeftMotor().setSpeed(FWD_SPEED);
    		getRightMotor().setSpeed(FWD_SPEED);
    		getLeftMotor().rotate(DriveManager.convertDistance(dist), true);
        getRightMotor().rotate(DriveManager.convertDistance(dist), false);  
        return;
    }
    
    /*
     * This method causes the robot to turn to the absolute heading
     */
    public void turnTo(double theta) {
    
    }
    
    /**
	 * This method causes the robot to travel to the absolute field location (x,y),
	 * specified in tile points.
	 * 
	 * @param x
	 * @param y
     * @throws OdometerExceptions 
	 */
	public void travelTo(double x, double y, boolean avoid) throws OdometerExceptions {
		
		SensorManager sensorManager = SensorManager.getInstance();
		
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
		turnBy(theta);

		// Calculate the distance the robot must travel to get to the waypoint
		double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
		
		// Start robot forward towards the waypoint
		forwardBy((int) distance);
		
		// Play sound when reaching location
		Sound.beep();
	}
	
	public void travelToGrid(double x, double y) throws OdometerExceptions {
		SensorManager sensorManager = SensorManager.getInstance();
		
		int curX = (int) Math.round(sensorManager.getOdometer().getXYT()[0] / DriveManager.TILE_SIZE);
		int curY = (int) Math.round(sensorManager.getOdometer().getXYT()[1] / DriveManager.TILE_SIZE);
		if((int) y != curY) {
			travelTo(curX, y, false);
			turnBy(90);
		}
		
		curY = (int) Math.round(sensorManager.getOdometer().getXYT()[1] / DriveManager.TILE_SIZE);
		curX = (int) Math.round(sensorManager.getOdometer().getXYT()[0] / DriveManager.TILE_SIZE);
		if((int) x != curX) {
			travelTo(x, curY, false);
		}
	}
	
    /**
	 * This method causes the robot to travel to the absolute field location (x,y),
	 * specified in tile points.
	 * 
	 * @param x
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
     * This method returns true if another thread has called travelTo() or 
     * turnTo() and the method has yet to return; false otherwise
     * 
     * @return
     * returns true if the robots motors are moving, false otherwise
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
		double width = DriveManager.TRACK;
		return convertDistance(Math.PI * width * angle / 360.0);
	}
	
	// Field Encapsulation
	
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
	 * @return the driveThread
	 */
	public DriveThread getDriveThread() {
		return driveThread;
	}

	/**
	 * @param driveThread the driveThread to set
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
	 * @param thread the thread to set
	 */
	private static void setThread(Thread thread) {
		DriveManager.thread = thread;
	}

}
