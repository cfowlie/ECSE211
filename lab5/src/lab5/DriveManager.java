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
	public static final double TRACK = 14.45;
	public static final int ROTATE_SPEED = 100;
	public static final int FWD_SPEED = 160;
	public static final double LIGHT_RADIUS = 13.5;

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
