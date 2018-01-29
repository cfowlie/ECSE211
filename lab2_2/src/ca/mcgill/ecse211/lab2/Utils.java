package ca.mcgill.ecse211.lab2;

public class Utils {

	  public static final double WHEEL_RAD = 2.09554;
	  public static final double WHEEL_TRACK = 9.2;	
	
	  /**
	   * This method allows the conversion of a distance to the total rotation of each wheel need to
	   * cover that distance.
	   * 
	   * @param radius
	   * @param distance
	   * @return
	   */
	  public static int convertDistance(double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
	  }

	  public static int convertAngle(double angle) {
	    return convertDistance(Math.PI * WHEEL_TRACK * angle / 360.0);
	  }
	  
	  public static double convertTacho(double tacho) {
		  return (Math.PI * tacho * WHEEL_RAD / 180);
	  }
	  
}
