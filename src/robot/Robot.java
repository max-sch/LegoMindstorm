package robot;

import actions.Actions;
import behaviour.Line;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorConstants;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;


public class Robot {

	final RegulatedMotor leftMotor = Motor.C;
	final RegulatedMotor rightMotor = Motor.B;
	final  EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	
	public void initializeMotors() {
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		leftMotor.rotateTo(0);
		rightMotor.rotateTo(0);
		leftMotor.setSpeed(400);
		rightMotor.setSpeed(400);
		leftMotor.setAcceleration(800);
		rightMotor.setAcceleration(800);
	}

	public Actions createActions() {
		return new Actions(rightMotor, leftMotor);
	}
	
	public void test() {
		//colorSensor.setFloodlight();
		

		while (true) {
			int samplesize = colorSensor.sampleSize();
			float[] samples = new float[samplesize];
			colorSensor.getRedMode().fetchSample(samples, 0);
		
	    	LCD.drawString("ID =" + samples[0],0,1); 
	    	
	    	if (Button.readButtons() != 0) {
	    		break;
	    	}
		}
	}
	
	public void test2() {
		colorSensor.setFloodlight(false);
	}
	
	public Line createNewLineBehavior() {
		return new Line (rightMotor, leftMotor, colorSensor);
	}
}
