/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package odometer;

import java.lang.Math;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import main.DriveManager;
import main.SensorManager;

public class Odometer extends OdometerData implements Runnable {
	
	

	static Odometer odo = null; // Returned as singleton

	// Motors and related variables
	private double leftMotorTachoCount;
	private double rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	public static double[] position = new double[3];

	private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

	/**
	 * This is the default constructor of this class. It initiates all motors and
	 * variables once.It cannot be accessed externally.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @throws OdometerExceptions
	 */
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD) throws OdometerExceptions {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// Reset the values of x, y and z to 0
		this.setXYT(0, 0, 0);

		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;

	}

	/**
	 * This method is meant to ensure only one instance of the odometer is used
	 * throughout the code.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {
		DriveManager driveManager = DriveManager.getInstance();
		
		if (odo != null) { // Return existing object
			return odo;
		} else { // create object and return it
			odo = new Odometer(driveManager.getLeftMotor(), driveManager.getRightMotor(), driveManager.widthCheck(), driveManager.WHEEL_RAD);
			return odo;
		}
	}

	/**
	 * This method is where the logic for the odometer will run. Use the methods
	 * provided from the OdometerData class to implement the odometer.
	 */
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();

			double leftInstantTacho = leftMotor.getTachoCount();
			double rightInstantTacho = rightMotor.getTachoCount();
			
			double dL = leftInstantTacho - leftMotorTachoCount;
			double dR = rightInstantTacho - rightMotorTachoCount;

			double d1 = (Math.PI * dL * DriveManager.WHEEL_RAD) / 180;
			double d2 = (Math.PI * dR * DriveManager.WHEEL_RAD) / 180;

			// Distance
			double distance = (d1 + d2) / 2;

			// Theta
			double dt = (d1 - d2) / DriveManager.widthCheck();

			position = odo.getXYT();
			// Positions
			double dx = distance * Math.sin((position[2]) * Math.PI / 180);
			double dy = distance * Math.cos((position[2]) * Math.PI / 180);

			leftMotorTachoCount = leftInstantTacho;
			rightMotorTachoCount = rightInstantTacho;

			odo.update(dx, dy, dt * 180 / Math.PI);

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}		
	}
	
	
	
	

}
