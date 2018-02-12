package lab4;

import java.util.ArrayList;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import ultrasonic.UltrasonicController;

public class UltrasonicLocalizer implements UltrasonicController {

	ArrayList<Integer> distances = new ArrayList<Integer>();
	
	double fallingEdge;
	double risingEdge;
	double estimate;
	
	int d;
	int k;
	
	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3UltrasonicSensor ultrasonicSensor; 

	
	UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor){
		// Motors
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
				
		// Distance and Noise Margin
		d = 5;
		k = 2;
	}
	
	public void fallingEdge() {
		
	}
	
	public void risingEdge() {
		// Begin Spinning
	}

	@Override
	public void processUSData(int distance) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int readUSDistance() {
		// TODO Auto-generated method stub
		return 0;
	}
	
}
