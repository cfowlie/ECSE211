/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab2.Lab3;
import ca.mcgill.ecse211.lab2.SquareDriver;

import java.lang.Math;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;

public class OdometryCorrection extends OdometerData implements Runnable {

  private static OdometryCorrection odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  
  public static final EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));

  private double[] position = new double[3];

  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  public OdometryCorrection(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    this.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static OdometryCorrection getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new OdometryCorrection(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static OdometryCorrection getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();
    
      double dL = leftMotor.getTachoCount() - leftMotorTachoCount;
      double dR = rightMotor.getTachoCount() - rightMotorTachoCount;
      
      double d1 = (Math.PI*dL*Lab3.WHEEL_RAD)/180;
    	  double d2 = (Math.PI*dR*Lab3.WHEEL_RAD)/180;
    	  
    	  //Distance 
    	  double distance = (d1+d2)/2;
    	  
    	  //Theta 
    	  double dt = (d2-d1)/Lab3.TRACK;
    	  
    	  position = odo.getXYT();
    	  //Positions 
    	  double dx = distance*Math.sin(position[2]+dt);
    	  double dy = distance*Math.cos(position[2]+dt);
      
    	  leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();

      odo.update(dx, dy, dt);
             
      // Detected Line
      if(colorSensor.getColorID() > 10) {

    	  	// Moving in X direction
    	  	if(dx > 0.05 || dx < -0.05) {
    	  	  	Sound.playTone(1500, 300);
        	  	odo.setX(SquareDriver.TILE_SIZE*Math.round(position[0]%SquareDriver.TILE_SIZE));
    	  	}
    	  	
    	  	// Moving in Y direction
    	  	if(dy > 0.05 || dy < -0.05) {
    	  		Sound.playTone(1000, 300);
        	  	odo.setY(SquareDriver.TILE_SIZE*Math.round(position[1]%SquareDriver.TILE_SIZE));
        	 }
    	  	
    	  	//Turning Correction
    	  	if(dt > 0.05 || dt < -0.05) {
    	  		//Nothing to do here
    	  	}
        	  	 
      }
      
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
