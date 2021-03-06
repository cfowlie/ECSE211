package odometer;

import java.text.DecimalFormat;

import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import light.ColorPoller;
import main.DriveManager;
import main.Main;
import main.SensorManager;
import ultrasonic.UltrasonicPoller;

/**
 * This class is used to display the content of the odometer variables (x, y,
 * Theta)
 * 
 * @author Connor Fowlie
 * @author Lucas Bluethner
 * @version Final 1.0.5
 */
public class Display implements Runnable {

	EV3UltrasonicSensor ultrasonicSensor;
	UltrasonicPoller usPoller;

	SensorManager sensorManager = SensorManager.getInstance();

	private Odometer odo;
	private TextLCD lcd;
	private double[] position;
	private final long DISPLAY_PERIOD = 25;
	private long timeout = Long.MAX_VALUE;

	/**
	 * This is the class constructor
	 * 
	 * @param odoData
	 * @throws OdometerExceptions
	 */
	public Display(TextLCD lcd) throws OdometerExceptions {
		odo = Odometer.getOdometer();
		this.lcd = lcd;

		this.usPoller = sensorManager.getUsPoller();
	}

	/**
	 * This is the overloaded class constructor
	 * 
	 * @param odoData
	 * @throws OdometerExceptions
	 */
	public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
		odo = Odometer.getOdometer();
		this.timeout = timeout;
		this.lcd = lcd;
	}

	/**
	 * This is the first method to run in this class when the thread begins.&nbsp;It
	 * clears the lcd screen, then continuously displays the current x, y, and theta
	 * position of the robot. 
	 */
	public void run() {

		SensorManager sensorManager = null;
		try {
			sensorManager = SensorManager.getInstance();
		} catch (OdometerExceptions e1) {
			e1.printStackTrace();
		}

		lcd.clear();

		long updateStart, updateEnd;

		long tStart = System.currentTimeMillis();
		do {
			updateStart = System.currentTimeMillis();

			// Retrieve x, y and Theta information
			position = odo.getXYT();

			// Print x,y, and theta information
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("X: " + numberFormat.format(position[0] / DriveManager.TILE_SIZE), 0, 0);
			lcd.drawString("Y: " + numberFormat.format(position[1] / DriveManager.TILE_SIZE), 0, 1);
			lcd.drawString("T: " + numberFormat.format((int) (position[2] % 360)), 0, 2);
			lcd.drawString("Wall: " + numberFormat.format(sensorManager.lightLeftPoller.getDiff()) + " "
					+ numberFormat.format(sensorManager.lightLeftPoller.getDiff()), 0, 3);
			lcd.clear(4); // always reload the color on the display

			if (sensorManager.getDistance() < 6) {
				lcd.drawString("Color:" + ColorPoller.color, 0, 4);
			}

			// this ensures that the data is updated only once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					e.printStackTrace();
				}

			}

		} while ((updateEnd - tStart) <= timeout);

	}

}
