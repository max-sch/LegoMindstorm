package behaviour;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;

public class Line {

	private boolean onLine = false;
	EV3ColorSensor colorSensor;
	RegulatedMotor leftMotor;
	RegulatedMotor rightMotor;
	
	
	public Line (RegulatedMotor rightMotor, RegulatedMotor leftMotor, EV3ColorSensor colorSensor) {
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.colorSensor = colorSensor;
	}	
	
	public void follow() {
		colorSensor.setFloodlight(true);
	
		float realTimeValue;
		float kp = 150;
		float initialValue = 0.05f;	
		float powerA = 0;
		float powerB = 0;
		float error = 0;
		float turn = 0;
		
		while (true) {
			realTimeValue = getRealTimeValue();
		
			if(realTimeValue < 0.1) {
				error = (realTimeValue - initialValue)*20;
			}
			else {
				error = (realTimeValue - initialValue)*3;
			}
			
			turn = kp * error;
			
			powerA = (125 + turn);
			powerB = (125 - turn);
			
			leftMotor.forward();
			rightMotor.forward();
			leftMotor.setSpeed((int) powerA);
			rightMotor.setSpeed((int) powerB);
			
		 	if (Button.readButtons() != 0) {
	    		break;
	    	}
		}
		
	}
	
	public float getRealTimeValue() {
		int samplesize = colorSensor.sampleSize();
		float[] samples = new float[samplesize];
		colorSensor.getRedMode().fetchSample(samples, 0);
		return samples[0];
	}

	public void searchLine() {
	
	}
}
