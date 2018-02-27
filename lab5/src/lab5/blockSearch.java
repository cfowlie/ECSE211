package lab5;



import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import light.ColorPoller;
import odometer.Odometer;
import odometer.OdometerExceptions;
import ultrasonic.UltrasonicPoller;


public class blockSearch  {

	private int color;
	private static int i =  0;

	
	

	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3UltrasonicSensor ultrasonicSensor;
	EV3UltrasonicSensor distanceSensor;
	UltrasonicPoller usPoller;
	UltrasonicPoller distancePoller;
	
	
	DriveManager driveManager = DriveManager.getInstance();
	SensorManager sensorManager = SensorManager.getInstance();

	
	blockSearch() throws OdometerExceptions {
		
		
		// Motors
				this.leftMotor = driveManager.getLeftMotor();
				this.rightMotor = driveManager.getRightMotor();
				
				// Sensors
				this.usPoller = sensorManager.getUsPoller();
				this.distancePoller = sensorManager.getDistancePoller();
	}

				
	public void bSearch() throws InterruptedException, OdometerExceptions {			
	
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		
		
		Thread.sleep(1000);
		
		leftMotor.setSpeed(DriveManager.ROTATE_SPEED);
		rightMotor.setSpeed(DriveManager.ROTATE_SPEED);
		
		double dx = Lab5.URx-Lab5.LLx;
		
		
		
		double DDy = (Lab5.URy-Lab5.LLy)*DriveManager.TILE_SIZE;
		
		double DDx = (Lab5.URx-Lab5.LLx)*DriveManager.TILE_SIZE;
		
		double xORy = 0;
		
		
		
		while(distancePoller.getDistance() > 40) {
			
			for(; i <= 3; i++) {
			
			if (i==0 || i==2) {
				xORy = DDy;
			} else {
				xORy = DDx;
				
			}
				
			leftMotor.rotate(DriveManager.convertDistance(xORy),true);
			rightMotor.rotate(DriveManager.convertDistance(xORy), false);
			
			leftMotor.rotate((int) (DriveManager.convertAngle(90)),true);
			rightMotor.rotate((int) (-DriveManager.convertAngle(90)), false);
			}	
			
		}
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		blockDetected(leftMotor,rightMotor);
	}
	
	
	void blockDetected(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) throws InterruptedException {
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		leftMotor.rotate((int) (DriveManager.convertAngle(90)),true);
		rightMotor.rotate((int) (-DriveManager.convertAngle(90)), false);
		
		double temp1x = Odometer.position[0];
		double temp1y = Odometer.position[1];
		
		while (usPoller.getDistance() > 10) {
			
			leftMotor.forward();
			rightMotor.forward();
			}
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		leftMotor.rotate(DriveManager.convertDistance(5),true);
		rightMotor.rotate(DriveManager.convertDistance(5), false);
		
		color = ColorPoller.getColorInt();
		
		if (color==Lab5.TB) {
			Sound.playTone(1000, 200);
			
		} else {
			Sound.playTone(300, 200);
			Thread.sleep(200);
			Sound.playTone(300, 200);
			
		}
		
		
		double temp2x = Odometer.position[0];
		double temp2y = Odometer.position[1];
		
		double distx = temp2x-temp1x;
		double disty = temp2y-temp1y;
		
		double stage_distance = Math.sqrt(Math.pow(distx, 2)+Math.pow(disty, 2));
		
		leftMotor.rotate((int) (-DriveManager.convertAngle(180)),true);
		rightMotor.rotate((int) (DriveManager.convertAngle(180)), false);
		
		leftMotor.rotate(DriveManager.convertDistance(stage_distance),true);
		rightMotor.rotate(DriveManager.convertDistance(stage_distance), false);
		
		leftMotor.rotate((int) (-DriveManager.convertAngle(90)),true);
		rightMotor.rotate((int) (DriveManager.convertAngle(90)), false);
		
	
		double regain_distance = Math.sqrt(Math.pow(Lab5.URy-temp1y, 2)+Math.pow(0, 2));
		
		leftMotor.rotate(DriveManager.convertDistance(regain_distance), true);
		rightMotor.rotate(DriveManager.convertDistance(regain_distance), false);
		
		leftMotor.rotate((int) (DriveManager.convertAngle(90)), true);
		rightMotor.rotate((int) (-DriveManager.convertAngle(90)), false);
		
		
	}
	
}
