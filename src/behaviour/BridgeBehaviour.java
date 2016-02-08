package behaviour;

import com.sun.prism.paint.Color;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import lejos.utility.Delay;
import robot.RobotConfiguration;

public class BridgeBehaviour implements IBehaviour {

	private final RegulatedMotor leftMotor;
	private final RegulatedMotor rightMotor;
	private final EV3UltrasonicSensor ultraSonicSensor;
	private final EV3ColorSensor colorSensor;
	private final RegulatedMotor ultraSonicMotor;
	private final EV3TouchSensor rightTouchSensor;
	private float currentDistance;
	
	private final static float minDistance = 0.2f;
	private final static int baseSpeed = 300;
	
	public boolean wasFound = false;
	
	public BridgeBehaviour(RobotConfiguration robotConf) {
		this.leftMotor = robotConf.getLeftMotor();
		this.rightMotor = robotConf.getRightMotor();
		this.ultraSonicSensor = robotConf.getUltraSonicSensor();
		this.colorSensor = robotConf.getColorSensor();
		this.ultraSonicMotor = robotConf.getUltraSonicMotor();
		this.rightTouchSensor = robotConf.getRightTouchSensor();
		
		this.ultraSonicMotor.rotate(0);
	}
	
	@Override
	public BarCode passObstacle() {
		ultraSonicMotor.rotate(-80);
		Delay.msDelay(500);
		
		//while (true) {
			findRightEdge();
			followRightEdge();
			
//			if (Button.readButtons() != 0) {
//				break;
//			}
		//}
		
		this.ultraSonicMotor.rotate(80);
		
		passElevator();
		
		//Has to be adapted
		return BarCode.FINISH;
	}
	
	private void passElevator() {
		this.leftMotor.stop();
		this.rightMotor.stop();
	}

//	private boolean isGroundColor() {
//		return (getColor() > 0.012f);
//	}

	private void followRightEdge() {
//		this.rightMotor.setSpeed(300);
//		this.leftMotor.setSpeed(100);
//		this.leftMotor.forward();
//		this.rightMotor.forward();
//		
//		Delay.msDelay(1000);
		float powerA;
		float powerB;
		float turn = 0;
		float kp;
		float error;
		float currentDistance;
		
		ScanRedAndGreenColor scan = new ScanRedAndGreenColor(colorSensor);
		
		while (true) {
			
			float val = scan.getColor();
			LCD.drawString("Val: " + val, 0, 1);
			if (val > 0.13f) {
				break;
			}
			
			currentDistance = getDistanceToGround();
			
			error = minDistance - currentDistance;
			if (error < -0.30f) {
				kp = 250;
			} else if ( error < 0) {
				kp = 800;
			} else {
				kp = 300;
			}
			
			
			turn = kp * error;
			
			powerA = baseSpeed + turn;
			powerB = baseSpeed - turn;
			
			leftMotor.setSpeed((int) powerA);
			rightMotor.setSpeed((int) powerB);
			leftMotor.forward();
			rightMotor.forward();
			
			if (Button.readButtons() != 0) {
				break;
			}
		}
	}

	private void findRightEdge() {
		this.rightMotor.setSpeed(280);
		this.leftMotor.setSpeed(300);
		this.leftMotor.forward();
		this.rightMotor.forward();
		
		currentDistance = getDistanceToGround();
		//LCD.drawString("val: " + currentDistance, 0, 1);
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
		this.ultraSonicSensor.fetchSample(samples, 0);
		
		return samples[0];
	}
	
	private boolean isRightTouched() {
		int sampleSize = this.rightTouchSensor.sampleSize();
		float[] samples = new float[sampleSize];
		this.rightTouchSensor.fetchSample(samples, 0);
		return samples[0] == 1;
	}
}
