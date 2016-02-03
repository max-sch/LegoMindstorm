package robot;

import actions.Actions;
import behaviour.Line;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.internal.ev3.EV3Battery;
import lejos.robotics.RegulatedMotor;


public class Robot {

	final RegulatedMotor leftMotor = Motor.C;
	final RegulatedMotor rightMotor = Motor.B;
	final  EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	final EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S3);
	final EV3TouchSensor leftTouchSensor = new EV3TouchSensor(SensorPort.S4);
	final EV3TouchSensor rightTouchSensor = new EV3TouchSensor(SensorPort.S1);
	
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
	
	public RegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}
	
	public RegulatedMotor getRightMotor() {
		return this.rightMotor;
	}
	
	public EV3UltrasonicSensor getUltraSonicSensor() {
		return this.ultrasonicSensor;
	}
	
	public EV3TouchSensor getLeftTouchSensor() {
		return this.leftTouchSensor;
	}
	
	public EV3TouchSensor getRightTouchSensor() {
		return this.rightTouchSensor;
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
	
	public void testLabySensors() {
		while (true) {
			int samplesize = this.leftTouchSensor.sampleSize();
			float[] samples = new float[samplesize];
			this.leftTouchSensor.fetchSample(samples, 0);
			
			LCD.drawString("ID =" + samples[0],0,1); 
	    	
	    	if (Button.readButtons() != 0) {
	    		break;
	    	}
		}
	}
	
	public void testUltraSonic() {
		while (true) {
			int samplesize = this.ultrasonicSensor.sampleSize();
			float[] samples = new float[samplesize];
			this.ultrasonicSensor.fetchSample(samples, 0);
		
	    	LCD.drawString("Val =" + samples[0],0,1); 
	    	
	    	if (Button.readButtons() != 0) {
	    		break;
	    	}
		}
	}
	
	public Line createNewLineBehavior() {
		return new Line (rightMotor, leftMotor, colorSensor);
	}
}
