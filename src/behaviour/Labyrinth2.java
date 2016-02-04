package behaviour;

import java.util.ArrayList;

import lejos.hardware.Button;
import lejos.hardware.Sound;
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
	private final EV3TouchSensor leftTouchSensor;

	float trend;
	float currentDistance = 0.05f;
	float oldDistance = 0.05f;
	float kp = 500f;
	float powerA = 0f;
	float powerB = 0f;
	float error = 0f;
	float turn = 0f;
	float baseSpeed = 600f;

	float averageDistance = 0.05f;


	public Labyrinth2(Robot robotConf) {
		this.leftMotor = robotConf.getLeftMotor();
		this.rightMotor = robotConf.getRightMotor();
		this.ultraSonicSensor = robotConf.getUltraSonicSensor();
		this.rightTouchSensor = robotConf.getRightTouchSensor();
		this.leftTouchSensor = robotConf.getLeftTouchSensor();
	}

	public void followRightWall() {
		currentDistance = getDistanceToWall();
		currentDistance = sanityCheck(currentDistance);
		while (true) {
			oldDistance = currentDistance;
			Delay.msDelay(500);
			currentDistance = getDistanceToWall();
			
			currentDistance = sanityCheck(currentDistance);
//			if (trend <= 0.001f && currentDistance < 0.06f ) {
//				powerA = 300;
//				powerB = 400;
//				leftMotor.forward();
//				rightMotor.forward();
//				leftMotor.setSpeed((int) powerA);
//				rightMotor.setSpeed((int) powerB);
//				Delay.msDelay(1000);
//				leftMotor.stop();
//				rightMotor.stop();
//				new DifferentialPilot(2, 2, leftMotor, rightMotor).rotate(60);
//			}
//			if (trend <= 0.001f && currentDistance > 0.10f) {
//				powerA = 400;
//				powerB = 375;
//				leftMotor.forward();
//				rightMotor.forward();
//				leftMotor.setSpeed((int) powerA);
//				rightMotor.setSpeed((int) powerB);
//				Delay.msDelay(1000);
//				leftMotor.stop();
//				rightMotor.stop();
//				new DifferentialPilot(2, 2, leftMotor, rightMotor).rotate(-60);
//			}		
				if (isRightTouched()) {
					turnRoutine();
				}

				trend = currentDistance - oldDistance;
				LCD.drawString("Val: " + trend, 0, 1);
				
				if ((trend < 0.003f && trend > -0.003f)) {
					if (currentDistance < 0.01f || currentDistance > 0.1f) {
						trend = trend * 1000;
					}
				} else {
					if (currentDistance < 0.05f) {
						turn =(5000f * trend);
					} else if (currentDistance > 0.07f) {
						turn = (5000f * trend);
					}else {
						turn =(kp * trend);
					}
				}

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
	
	private void turnRoutine() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		
		DifferentialPilot pilot = new DifferentialPilot(2, 2, leftMotor, rightMotor);
		pilot.backward();
		Delay.msDelay(1500);
		this.ultraSonicSensor.disable();
		pilot.stop();
		pilot.rotate(330);
		this.ultraSonicSensor.enable();
	}
	
	private boolean isRightTouched() {
		int sampleSize = this.rightTouchSensor.sampleSize();
		float[] samples = new float[sampleSize];
		this.rightTouchSensor.fetchSample(samples, 0);
		return samples[0] == 1;
	}

	private float getDistanceToWall() {
		int sampleSize = this.ultraSonicSensor.sampleSize();
		float[] samples = new float[sampleSize];
		ultraSonicSensor.fetchSample(samples, 0);
		return samples[0];
	}

	private float sanityCheck(float currentDistance) {
		if (currentDistance >= 0.70f) {
			return 0.20f;
		}
		return currentDistance;
	}

}
