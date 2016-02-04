package behaviour;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;

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
		int baseSpeed = 125;
		
		while (true) {
			realTimeValue = getRealTimeValue();
			baseSpeed = 125;
			if(realTimeValue < 0.1) {
				error = (realTimeValue - initialValue)*20;
			}
			else if (realTimeValue > 0.5) {
				baseSpeed = 0;
				leftMotor.stop();
				rightMotor.stop();
				new DifferentialPilot(2, 2, leftMotor, rightMotor).rotate(-60);
			}
			else {
				error = (realTimeValue - initialValue)*3;
			}
			
			turn = kp * error;
			
			powerA = (baseSpeed + turn);
			powerB = (baseSpeed - turn);
			
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
