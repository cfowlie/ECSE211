package ca.mcgill.ecse211.lab2;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class Navigation {

	private static final int FORWARD_SPEED = 185;
	  private static final int ROTATE_SPEED = 85;
	  public static final double TILE_SIZE = 30.48;	
	
	  public static final int map1_X[] = {0,1,2,2,1};
	  public static final int map2_X[] = {1,0,2,2,1};
	  public static final int map3_X[] = {1,2,2,0,1};
	  public static final int map4_X[] = {0,1,1,2,2};
	  
	  public static final int map1_Y[] = {2,1,2,1,0};
	  public static final int map2_Y[] = {1,2,2,1,0};
	  public static final int map3_Y[] = {0,1,2,2,1};
	  public static final int map4_Y[] = {1,2,0,1,2};
	  
	  
	  public static final int all_maps_X[][]= {map1_X,map2_X,map3_X,map4_X};
	  public static final int all_maps_Y[][]= {map1_Y,map2_Y,map3_Y,map4_Y};
	
	  

	  private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
		  }

		  private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
		  }
	  
	  public static double travelDistance(int stage, int map) {
		 
		 double distance = Math.sqrt(Math.pow((double)(Odometer.position[0]-TILE_SIZE*all_maps_X[map][stage]), 2)+Math.pow((double)(Odometer.position[1]-TILE_SIZE*all_maps_Y[map][stage]), 2));
		 
		 return distance;
	  }
	  
	  public static double turnAngle(int stage, int map) {
		  
		  double angle = Math.atan((double) Odometer.position[0]-TILE_SIZE*all_maps_X[map][stage]/ ((double) Odometer.position[1]-TILE_SIZE*all_maps_Y[map][stage]+0.0001));
			  
		  return angle;
	  }

	  
	  public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		      double leftRadius, double rightRadius, double track, int map) {
		  
		  
		    // reset the motors
		    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.stop();
		      motor.setAcceleration(1000);
		      
		      
		      try {
		          Thread.sleep(2000);
		        } catch (InterruptedException e) {
		          // There is nothing to be done here
		        }

		      
		      for (int i=0; i<5; i++) {
		    	  
		    	  double f = turnAngle(i,map);
		    	  
		    	  leftMotor.setSpeed(ROTATE_SPEED);
		          rightMotor.setSpeed(ROTATE_SPEED);
		    	  
		    	  leftMotor.rotate(convertAngle(leftRadius, track, f), true);
		          rightMotor.rotate(-convertAngle(rightRadius, track, f), false);
		          
		          leftMotor.setSpeed(FORWARD_SPEED);
		          rightMotor.setSpeed(FORWARD_SPEED);

		          leftMotor.rotate(convertDistance(leftRadius, travelDistance(i,map)), true);
		          rightMotor.rotate(convertDistance(rightRadius, travelDistance(i,map)), false);
		    	  
		      }
		      
		      
		      
		    }
	  }
}
