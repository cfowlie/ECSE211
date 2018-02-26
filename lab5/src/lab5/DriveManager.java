package lab5;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class DriveManager {

	// Singleton Object
	private static DriveManager sharedManager = null;

	// Motor Objects
	private final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	// Constants
	public static final double WHEEL_RAD = 2.095;
	public static final double TRACK = 17.8;
	public static final int ROTATE_SPEED = 60;
	public static final int FWD_SPEED = 100;
	public static final double LIGHT_RADIUS = 13.4;

	protected DriveManager() {

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

}
