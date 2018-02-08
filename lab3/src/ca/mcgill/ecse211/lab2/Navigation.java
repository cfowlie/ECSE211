package ca.mcgill.ecse211.lab2;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.UltrasonicController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;


public class Navigation{

	private static final int FORWARD_SPEED = 185;
	private static final int ROTATE_SPEED = 50;
	public static final double TILE_SIZE = 30.48;	

	int distance;
	
	static double arr_d[] = new double[6];

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

	/*
	 * Returns updated heading in RAD
	 */
	 public static double getHeading(int stage, int map) {
		 int dx,dy = 0;
		 

		 if(stage > 0) {
			 int x1 = all_maps_X[map][stage-1];
			 int x2 = all_maps_X[map][stage];
			 dx = x2-x1;

			 int y1 = all_maps_Y[map][stage-1];
			 int y2 = all_maps_Y[map][stage];
			 dy = y2-y1;	
		 }
		 else {
			 dx = all_maps_X[map][stage];
			 dy = all_maps_Y[map][stage];
		 }

		 if(dx==0) {
			 if(dy>0) {
				 return 0;
			 }
			 return Math.PI;
		 }

		 double theta = Math.atan(dy/dx); //t is the new heading

		 if(theta < 0) { // Convert negative values to positive
			 theta = (Math.PI*2)+Math.atan(dy/dx);
		 }

		 if(dx < 0) { // Atan correction for negative values
			 theta += Math.PI;
		 }

		 if(theta > Math.PI*2) { // Convert values over 360 degrees
			 theta-= Math.PI*2;
		 }

		 double odoHeading = (5*Math.PI/2) - theta; //Convert to odo heading

		 if(odoHeading > Math.PI*2) { // Convert values over 360 degrees
			 odoHeading-= Math.PI*2;
		 }

		 return odoHeading;
	 }

	 /*
	  * Returns angle needed in degrees
	  */
	 public static double turnAngle(int stage, int map) {

		 double odoHeading = getHeading(stage,map);

		 double currentHeading = Math.toRadians(Odometer.position[2]);

		 double angle = odoHeading - currentHeading;

		 if(angle < -Math.PI) {
			 angle += 2*Math.PI;
		 }
		 else if (angle >= Math.PI){
			 angle -= 2*Math.PI;
		 }

		 return Math.toDegrees(angle);
	 }


	 public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			 double leftRadius, double rightRadius, double track, int map) {


		 // reset the motors
		 for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			 motor.stop();
			 motor.setAcceleration(3000);
		 }

		 try {
			 Thread.sleep(8000);
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
	 
	

	public static void processUSData(int distance) {
	    if(distance <= 15) {
	    	Lab3.rightMotor.stop();
	    	Lab3.leftMotor.stop();
	    	System.exit(0);
	    }
	}

	
	
	  public int readUSDistance() {
		    return this.distance;
		  }
		

}
