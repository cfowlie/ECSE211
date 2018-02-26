package lab5;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

interface DriveThread {
	
	/*
	 * Run robot code on async thread
	 */
	void run() throws InterruptedException;
	
	/*
	 * Completion method for callback
	 */
	void completion();
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
	public static final double WHEEL_RAD = 2.095;
	public static final double TRACK = 17.15;
	public static final int ROTATE_SPEED = 100;
	public static final int FWD_SPEED = 140;
	public static final double LIGHT_RADIUS = 13.5;
	public static final double NO_WALL_DIST = 35;
	public static final double TILE_SIZE = 30.48;


	private DriveManager() {
		setThread((new Thread() {
			public void run() {
				try {
					DriveManager.driveThread.run();
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
				}
			}
		}));
	}
	
	public void start() {
		this.getThread().start();	
	}

	public static DriveManager getInstance() {
		if (sharedManager == null) {
			sharedManager = new DriveManager();
		}
		return sharedManager;
	}
	
	/**
     * This method causes the robot to turn (on point) by amount theta
     * 
     * @param theta
     */
    public void turnBy(double theta) {
    		DriveManager.getInstance().getLeftMotor().setSpeed(ROTATE_SPEED);
    		DriveManager.getInstance().getRightMotor().setSpeed(ROTATE_SPEED);
        DriveManager.getInstance().getLeftMotor().rotate(DriveManager.convertAngle(theta), true);
        DriveManager.getInstance().getRightMotor().rotate(-DriveManager.convertAngle(theta), false);  
        return;
    }
    
	/**
     * This method causes the robot to turn (on point) by amount theta
     * 
     * @param theta
     */
    public void forwardBy(int distance) {
    		DriveManager.getInstance().getLeftMotor().setSpeed(FWD_SPEED);
    		DriveManager.getInstance().getRightMotor().setSpeed(FWD_SPEED);
        DriveManager.getInstance().getLeftMotor().rotate(distance, true);
        DriveManager.getInstance().getRightMotor().rotate(distance, false);  
        return;
    }
    
    /*
     * This method causes the robot to turn to the absolute heading
     */
    public void turnTo(double theta) {
    
    }
    
    public  void stopAll() {
    		DriveManager.getInstance().getLeftMotor().stop(true);
		DriveManager.getInstance().getRightMotor().stop(false);
    }
    
    /**
     * This method returns true if another thread has called travelTo() or 
     * turnTo() and the method has yet to return; false otherwise
     * 
     * @return
     * returns true if the robots motors are moving, false otherwise
     */
    boolean isNavigating() {
        return leftMotor.isMoving() && rightMotor.isMoving();
    }

	public static int convertDistance(double distance) {
		double radius = DriveManager.WHEEL_RAD;
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

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
