package odometer;

import java.text.DecimalFormat;

import lab5.SensorManager;
import lejos.hardware.lcd.TextLCD;
import light.ColorPoller;

/**
 * This class is used to display the content of the odometer variables (x, y,
 * Theta)
 */
public class Display implements Runnable {

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

	public void run() {

		SensorManager sensorManager = null;
		try {
			sensorManager = SensorManager.getInstance();
		} catch (OdometerExceptions e1) {
			// TODO Auto-generated catch block
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
			lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
			lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
			lcd.drawString("T: " + numberFormat.format((int) (position[2] % 360)), 0, 2);
			
			lcd.clear(3); //always reload the color on the display
			lcd.clear(4); //always reload the color on the display
			
			if(sensorManager.getDistance() < 6) {
				lcd.drawString("Object Detected", 0, 3);
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
