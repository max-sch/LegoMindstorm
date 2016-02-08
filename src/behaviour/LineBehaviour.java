package behaviour;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import lejos.robotics.navigation.DifferentialPilot;
import robot.RobotConfiguration;

public class LineBehaviour implements IBehaviour {

	EV3ColorSensor colorSensor;
	RegulatedMotor leftMotor;
	RegulatedMotor rightMotor;
	MedianFilter filter;
	private int barsCounted = 0;
	private boolean followLine = true;
	private BarCounterThread barCounter;
	
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
		barCounter = new BarCounterThread(this, colorSensor);
		barCounter.start();
		while (followLine) {
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
		 		Button.waitForAnyPress();
	    		break;
	    	}
		}
		leftMotor.stop();
		rightMotor.stop();
		// TODO has to be adapted
		return BarCode.FINISH;
	}
	
	public float getRealTimeValue() {
		int samplesize = filter.sampleSize();
		float[] samples = new float[samplesize];
		filter.fetchSample(samples, 0);
		return samples[0];
	}

	public void stopFollowLine() {
		followLine = false;
	}
	
	public void checkIfSharpCorner() {
		boolean isSharpCorner = false;
		int performedRotation = 0;
		DifferentialPilot pilot = new DifferentialPilot(2, 10, leftMotor, rightMotor);
		if (getRealTimeValue() > 0.2) {
			pilot.rotate(-2);
			isSharpCorner = true;
			barCounter.isSharpCorner();
			followLine = true;
		} else {
			for (int i = 0;  i<= 3 ; i++) {
				pilot.rotate(2);	
				performedRotation += 2;
				if (getRealTimeValue() > 0.05) {
					isSharpCorner = true;
					barCounter.isSharpCorner();
					followLine = true;
					break;
				}
			 	if (Button.readButtons() != 0) {
		    		break;
		    	}
			}
			if (!isSharpCorner) {
				pilot.rotate(-performedRotation);
				pilot.rotate(-20);
				barCounter.setIsNoSharpCorner();
			} 
		}
	}
	
	public void barFound() {
		barsCounted++;
		Sound.buzz();
	}
}
