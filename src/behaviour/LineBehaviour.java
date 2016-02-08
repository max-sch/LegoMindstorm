package behaviour;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import robot.RobotConfiguration;

public class LineBehaviour implements IBehaviour {

	EV3ColorSensor colorSensor;
	RegulatedMotor leftMotor;
	RegulatedMotor rightMotor;
	MedianFilter filter;
	private int barsCounted = 0;
	
	public LineBehaviour(RobotConfiguration robotConfig) {
		this.rightMotor = robotConfig.getRightMotor();
		this.leftMotor = robotConfig.getLeftMotor();
		this.colorSensor = robotConfig.getColorSensor();
		
		SampleProvider light = colorSensor.getMode("Red");
		filter = new MedianFilter(light, 10);
	}	
	
	@Override
	public BarCode passObstacle() {
		colorSensor.setFloodlight(true);
		float realTimeValue;
		float kp = 160; 
		float initialValue = 0.05f;	
		float powerA = 0;
		float powerB = 0;
		float error = 0;
		float turn = 0;
		int baseSpeed = 125;
		BarCounterThread barCounter = new BarCounterThread(this, colorSensor);
		barCounter.start();
		while (true) {
			realTimeValue = getRealTimeValue();
			baseSpeed = 220;
			if(realTimeValue < 0.1) {
				error = (realTimeValue - initialValue)*20;
			}
			else {
				error = (realTimeValue - initialValue)*5;
			}
			turn = kp * error;
			powerA = (baseSpeed + turn);
			powerB = (baseSpeed - turn);
			leftMotor.forward();
			rightMotor.forward();
			leftMotor.setSpeed((int) powerA);
			rightMotor.setSpeed((int) powerB);
		 	if (Button.readButtons() != 0) {
		 		barCounter.interrupt();
		 		System.out.println("Bars:"+ barsCounted);
		 		Button.waitForAnyPress();
	    		break;
	    	}
		}
		
		// TODO has to be adapted
		return BarCode.FINISH;
	}
	
	public float getRealTimeValue() {
		int samplesize = filter.sampleSize();
		float[] samples = new float[samplesize];
		filter.fetchSample(samples, 0);
		return samples[0];
	}

	public void barFound() {
		barsCounted++;
		Sound.buzz();
	}
}
