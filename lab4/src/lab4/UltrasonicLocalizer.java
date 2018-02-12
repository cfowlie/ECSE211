package lab4;

import java.util.ArrayList;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class UltrasonicLocalizer {

	ArrayList<Integer> distances = new ArrayList<Integer>();
	
	double fallingEdge;
	double risingEdge;
	double estimate;
	
	int d;
	int k;
	
	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	EV3UltrasonicSensor ultrasonicSensor; 

	
	UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3UltrasonicSensor ultrasonicSensor){
		// Motors
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		// Sensors
		this.ultrasonicSensor = ultrasonicSensor;
		
		// Distance and Noise Margin
		d = 5;
		k = 2;
	}
	
	public void fallingEdge() {
		
	}
	
	public void risingEdge() {
		// Begin Spinning
	}
	
}
