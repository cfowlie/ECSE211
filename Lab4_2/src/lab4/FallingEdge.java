package lab4;
import java.util.ArrayList;

import lejos.hardware.motor.*;
import odometer.Odometer;

public class FallingEdge implements UltrasonicController {

  private final int d;
  private final int k;
  
  private static double t1;
  private static double t2;
  
  private final int motorHigh;
  private static int distance;


  
  ArrayList<Integer> distances = new ArrayList<Integer>();

  public FallingEdge(int d, int k, int motorHigh) {
    // Default Constructor
    this.d = d;
    this.k = k;
    this.motorHigh = motorHigh;
    
  }
  
  
  public static void Fall(){
	
	  
	  
	  Lab4.leftMotor.setSpeed(100);
	    Lab4.rightMotor.setSpeed(100);
	  
	  if(distance<100) {
		  while(distance<100) {
			  Lab4.leftMotor.forward();
			  Lab4.rightMotor.backward();
		  }	  
	  }
	  
	  
	 
	    
	    while(distance > 100) {
	    
	    Lab4.leftMotor.forward();
	    Lab4.rightMotor.backward();
	    
	    
	    }

	    	 Lab4.leftMotor.stop();
	    	 Lab4.rightMotor.stop();
	    	 
	    	 try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
				}
		    	 t1 = Odometer.position[2];
	    	 
	    	 while(distance < 100) {
	    		 
	    		 Lab4.leftMotor.backward();
	    		 Lab4.rightMotor.forward();
	    		 
	    	 }
	    	 
	    while(distance > 100) {
		    
		    Lab4.leftMotor.backward();
		    Lab4.rightMotor.forward();
		    
		    
		    }
	    
	  
	 Lab4.leftMotor.stop();
   	 Lab4.rightMotor.stop();
   	 
   	 try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
			}
	    	 t2 = Odometer.position[2];
	    	 
	    double dt = t2-t1;

	    if(dt < -180) {
			 dt += 360;
		 }
		 else if (dt >= 180){
			 dt -= 360;
		 }
	    
	    double angle = dt/2 + 45;
	    
	    Lab4.leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, angle));
	    Lab4.rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, angle));
	    
	    Odometer.odo.setTheta((double) 0.0);
  }

  @SuppressWarnings("static-access")
@Override
  public void processUSData(int distance) {
    this.distance = distance;
    
    try {
		Thread.sleep(25);
	} catch (InterruptedException e) {
		
	}
    
   distances.add(distance);
    
    }
  
  

  @SuppressWarnings("static-access")
@Override
  public int readUSDistance() {
    return this.distance;
  }
  
  private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	  }

	  private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	  }
}
