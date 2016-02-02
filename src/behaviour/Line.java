package behaviour;

import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.NXTLightSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;

import java.awt.image.ColorConvertOp;

import com.sun.prism.paint.Color;

import lejos.hardware.Button;

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
		//colorSensor.setFloodlight(lejos.robotics.Color.RED);
		int vr;
		int vl;
		float realTimeValue;
		int kp = 100;
		float initialValue = 50f;
	
		while (true) {
			realTimeValue = getRealTimeValue() * 100;
			float error = realTimeValue - initialValue;
			float turn = kp * error;
			turn = turn/100;
			float powerA = (initialValue +turn)*4.0f;
			float powerB = (initialValue -turn)*4.0f;
//			vl = (int) (160 + turn);
//			vr = (int) (160 - turn);
			leftMotor.backward();
			rightMotor.backward();
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

//	public void follow() {
//		leftMotor.setSpeed(200);
//		rightMotor.setSpeed(200);
//		leftMotor.forward();
//		rightMotor.forward();
//		while(onLine) {
//			if (colorSensor.getColorID() != 0) {
//				onLine = false;
//				leftMotor.stop();
//				rightMotor.stop();
//				searchAfterLineWasLost();
//			}
//			if (Button.readButtons() != 0) {
//				break;
//			}
//		}
//		
//	}

	public void searchAfterLineWasLost() {
	
	}
}
