package behaviour;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import robot.RobotConfiguration;

public class LineBehaviourThread extends Thread {
	
	private final RegulatedMotor rightMotor;
	private final RegulatedMotor leftMotor;
	private final EV3ColorSensor colorSensor;
	private MedianFilter filter;
	private SampleProvider provider;
	
	private boolean isStopped;
	
	public LineBehaviourThread(RobotConfiguration robotConfigs) {
		this.rightMotor = robotConfigs.getRightMotor();
		this.leftMotor = robotConfigs.getLeftMotor();
		this.colorSensor = robotConfigs.getColorSensor();
		this.provider = this.colorSensor.getRedMode();
		this.filter = new MedianFilter(provider, 10);
		
		isStopped = false;
	}
	
	@Override
	public void run() {
		float realTimeValue, error, turn, powerA, powerB;
		int baseSpeed;
		float kp = 160; 
		float initialValue = 0.05f;	
		
		
		while (!isStopped) {
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
		 		Button.waitForAnyPress();
	    		break;
	    	}
		}
	}

	public boolean isStopped() {
		return isStopped;
	}

	public void setStopped(boolean isStopped) {
		this.isStopped = isStopped;
	}
	
	private float getRealTimeValue() {
		int samplesize = this.filter.sampleSize();
		float[] samples = new float[samplesize];
		this.filter.fetchSample(samples, 0);
		return samples[0];
	}
}
