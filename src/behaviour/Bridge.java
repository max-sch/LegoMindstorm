package behaviour;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import robot.Robot;

public class Bridge {

	private final RegulatedMotor leftMotor;
	private final RegulatedMotor rightMotor;
	private final EV3UltrasonicSensor ultraSonicSensor;
	private final EV3ColorSensor colorSensor;
	private final RegulatedMotor ultraSonicMotor;
	private final EV3TouchSensor rightTouchSensor;
	private final MedianFilter filter;
	private float currentDistance;
	
	private final static float minDistance = 0.2f;
	
	public Bridge(Robot robotConf) {
		this.leftMotor = robotConf.getLeftMotor();
		this.rightMotor = robotConf.getRightMotor();
		this.ultraSonicSensor = robotConf.getUltraSonicSensor();
		this.colorSensor = robotConf.getColorSensor();
		this.ultraSonicMotor = robotConf.getUltraSonicMotor();
		this.rightTouchSensor = robotConf.getRightTouchSensor();
		SampleProvider provider = this.ultraSonicSensor.getDistanceMode();
		this.filter = new MedianFilter(provider, 10);
	}
	
	public void passBridge() {
		ultraSonicMotor.rotate(-60);
		Delay.msDelay(500);
		
		while (!isRightTouched()) {
			findRightEdge();
			followRightEdge();
			
			if (Button.readButtons() != 0) {
				break;
			}
		}
		
		this.ultraSonicMotor.rotate(60);
	}
	
	private void followRightEdge() {
		this.rightMotor.setSpeed(300);
		this.leftMotor.setSpeed(100);
		this.leftMotor.forward();
		this.rightMotor.forward();
		
		Delay.msDelay(1000);
	}

	private void findRightEdge() {
		this.rightMotor.setSpeed(200);
		this.leftMotor.setSpeed(300);
		this.leftMotor.forward();
		this.rightMotor.forward();
		
		currentDistance = getDistanceToGround();
		LCD.drawString("val: " + currentDistance, 0, 1);
		while (currentDistance < minDistance) {
			currentDistance = getDistanceToGround();
			
			if (Button.readButtons() != 0) {
				break;
			}
		}
		
		//LCD.drawString("val: " + currentDistance, 0, 1);
//		this.leftMotor.stop();
//		this.rightMotor.stop();
//		Delay.msDelay(500);
		
	}
	
	private float getDistanceToGround() {
		int sampleSize = this.ultraSonicSensor.sampleSize();
		float[] samples = new float[sampleSize];
		//filter.fetchSample(samples, 0);
		this.ultraSonicSensor.fetchSample(samples, 0);
		return samples[0];
	}
	
	private float getColor() {
		int samplesize = colorSensor.sampleSize();
		float[] samples = new float[samplesize];
		colorSensor.getRedMode().fetchSample(samples, 0);
		return samples[0];
	}
	
	private boolean isRightTouched() {
		int sampleSize = this.rightTouchSensor.sampleSize();
		float[] samples = new float[sampleSize];
		this.rightTouchSensor.fetchSample(samples, 0);
		return samples[0] == 1;
	}
}
