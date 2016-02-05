package behaviour;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import robot.Robot;

public class Labyrinth2 {

	private final RegulatedMotor leftMotor;
	private final RegulatedMotor rightMotor;
	private final EV3UltrasonicSensor ultraSonicSensor;
	private final EV3TouchSensor rightTouchSensor;
	//private final EV3TouchSensor leftTouchSensor;
	int baseSpeed = 650;
//	float trend;
//	float currentDistance = 0.05f;
//	float oldDistance = 0.05f;
//	float kp = 500f;
//	float powerA = 0f;
//	float powerB = 0f;
//	float error = 0f;
//	float turn = 0f;
//	int baseSpeed = 300;
//
//	float averageDistance = 0.05f;


	public Labyrinth2(Robot robotConf) {
		this.leftMotor = robotConf.getLeftMotor();
		this.rightMotor = robotConf.getRightMotor();
		this.ultraSonicSensor = robotConf.getUltraSonicSensor();
		this.rightTouchSensor = robotConf.getRightTouchSensor();
		//this.leftTouchSensor = robotConf.getLeftTouchSensor();
	}

//	public void followRightWall() {
//		float trend;
//		float currentDistance = 0.05f;
//		float oldDistance = 0.05f;
//		float kp = 500f;
//		float powerA = 0f;
//		float powerB = 0f;
//		float error = 0f;
//		float turn = 0f;
//		int baseSpeed = 300;
//		currentDistance = getDistanceToWall();
//		currentDistance = sanityCheck(currentDistance);
//		while (true) {
//			oldDistance = currentDistance;
//			Delay.msDelay(500);
//			currentDistance = getDistanceToWall();
//			
//			currentDistance = sanityCheck(currentDistance);
//			
//			if (isRightTouched()) {
//				turnRoutine();
//			}
//
//			trend = currentDistance - oldDistance;
//			LCD.drawString("Val: " + trend, 0, 1);
//				
//			if ((trend < 0.003f && trend > -0.003f)) {
//				if (currentDistance < 0.01f || currentDistance > 0.1f) {
//					trend = trend * 1000;
//				}
//			} else {
//				if (currentDistance < 0.05f) {
//					turn =(5000f * trend);
//				} else if (currentDistance > 0.07f) {
//					turn = (5000f * trend);
//				}else {
//					turn =(kp * trend);
//				}
//			}
//
//			powerA = (baseSpeed + turn);
//			powerB = (baseSpeed - turn);
//			leftMotor.forward();
//			rightMotor.forward();
//			leftMotor.setSpeed((int) powerA);
//			rightMotor.setSpeed((int) powerB);
//				
//			if (Button.readButtons() != 0) {
//				break;
//			}
//		
//		}
//	}
	
	public void followRightWall2() {
		float currentDistance;
		float oldDistance;
		float trend;
		
		this.leftMotor.setSpeed(baseSpeed);
		this.rightMotor.setSpeed(baseSpeed);
		this.leftMotor.forward();
		this.rightMotor.forward();
		
		oldDistance = getDistanceToWall();
		
		while (true) {
			currentDistance = getDistanceToWall();
			trend = currentDistance - oldDistance;
			LCD.drawString("val: " + trend, 0, 1);
			if (currentDistance <= 0.05f) {
				//float calc = 25 - (5 - (currentDistance*100));
				float calc = 100 - (5 - (currentDistance*100));
				onRev(baseSpeed, true);
				onRev((int) calc, false);
				
				this.leftMotor.forward();
				this.rightMotor.forward();
			} else if (currentDistance > 0.05f && currentDistance <= 0.1f) {
				onRev(baseSpeed, true);
				onRev(baseSpeed, false);
				this.leftMotor.forward();
				this.rightMotor.forward();
			} else if (currentDistance > 0.1f && currentDistance < 0.2f) {
				//float calc = 30 - ((currentDistance*100) - 10);
				float calc = 100 - ((currentDistance*100) - 10);
				
				onRev((int) calc, true);
				onRev(baseSpeed, false);
				
				this.leftMotor.forward();
				this.rightMotor.forward();
			}
			else if (currentDistance > 0.2f) {
				
				onRev(300, true);
				onRev(baseSpeed, false);
				
				this.leftMotor.forward();
				this.rightMotor.forward();
			}
			
			if (isRightTouched()) {
				turnRoutine();
			}
			
			oldDistance = currentDistance;
			
			if (Button.readButtons() != 0) {
				break;
			}
		
		}
		
	}
	
	private void secondAttempt(float currentDistance) {
		float oldDistance = currentDistance;
		currentDistance = getDistanceToWall();
		if (currentDistance > 0.5f) {
			currentDistance = 0.5f;
		}
		
		float trend = currentDistance - oldDistance;
		LCD.drawString("Val: "+trend, 0, 1);
		
		if (trend < 0f) {
			onRev(400, true);
			onRev(150, false);
		} else {
			onRev(150, true);
			//onRev((int) calc, true);
			onRev(400, false);
		}
	}
	
//	private void testedCurve() {
//		DifferentialPilot pilot = new DifferentialPilot(2, 10, leftMotor, rightMotor);
//		pilot.stop();
//		// 90° -> 2, 10
//		pilot.rotate(-30);
//		
//		this.leftMotor.setSpeed(baseSpeed);
//		this.rightMotor.setSpeed(baseSpeed);
//		this.leftMotor.forward();
//		this.rightMotor.forward();
//		
//		Delay.msDelay(1200);
//	}
	
//	private void stopMotors() {
//		this.leftMotor.stop();
//		this.rightMotor.stop();
//	}
	
	private void onRev(int Speed, boolean isRightMotor) {
		if (isRightMotor) {
			this.rightMotor.setSpeed(Speed);
		} else {
			this.leftMotor.setSpeed(Speed);
		}
	}
	
	private void turnRoutine() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		
		DifferentialPilot pilot = new DifferentialPilot(2, 2, leftMotor, rightMotor);
		pilot.backward();
		Delay.msDelay(300);
		this.ultraSonicSensor.disable();
		pilot.stop();
		pilot.rotate(330);
		this.ultraSonicSensor.enable();
	}
//	
	private boolean isRightTouched() {
		int sampleSize = this.rightTouchSensor.sampleSize();
		float[] samples = new float[sampleSize];
		this.rightTouchSensor.fetchSample(samples, 0);
		return samples[0] == 1;
	}
//
	private float getDistanceToWall() {
		int sampleSize = this.ultraSonicSensor.sampleSize();
		float[] samples = new float[sampleSize];
		ultraSonicSensor.fetchSample(samples, 0);
		return samples[0];
	}
//
//	private float sanityCheck(float currentDistance) {
//		if (currentDistance >= 0.70f) {
//			return 0.20f;
//		}
//		return currentDistance;
//	}

}
