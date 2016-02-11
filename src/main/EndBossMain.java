package main;

import behaviour.BarCode;
import lejos.hardware.Button;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.filter.MedianFilter;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import robot.Robot;
import robot.RobotConfiguration;

public class EndBossMain {
	
	private final static int baseSpeed = 650;
	private final static float minDistToWall = 0.05f;
	private final static float midDistToWall = 0.1f;
	private final static float maxDistToWall = 0.2f;
	
	private static MedianFilter filter;
	private static RegulatedMotor rightMotor;
	private static RegulatedMotor leftMotor;
	private static EV3UltrasonicSensor ultraSonicSensor;
	private static EV3TouchSensor rightTouchSensor;
	
	public static void main(String[] args) {
		RobotConfiguration robotConfig = new RobotConfiguration();
		Robot robot = new Robot(robotConfig);
		robotConfig.initializeMotors();
		
		rightTouchSensor = robotConfig.getLeftTouchSensor();
		
		Button.LEDPattern(6);
		Button.waitForAnyPress();
		
		DifferentialPilot pilot = new DifferentialPilot(2, 10, robotConfig.getLeftMotor(), robotConfig.getRightMotor());
		pilot.setTravelSpeed(800);
		pilot.forward();
		
		while (true) {
			if (Button.readButtons() != 0) {
	    		break;
	    	}
			
			if (isLeftTouched()) {
				pilot.stop();
				pilot.rotate(-40);
				robot.passObstacleWithBarCode(BarCode.LABYRINTH);
			}
		}
		
		pilot.stop();
		//robot.passObstacleWithBarCode(BarCode.LABYRINTH);
	}

	private static void onRev(int Speed, boolean isRightMotor) {
		if (isRightMotor) {
			rightMotor.setSpeed(Speed);
		} else {
			leftMotor.setSpeed(Speed);
		}
	}
	
	private static float getRealTimeValue() {
		int samplesize = filter.sampleSize();
		float[] samples = new float[samplesize];
		filter.fetchSample(samples, 0);
		return samples[0];
	}
	
	private static void turnRoutine() {
		leftMotor.stop();
		rightMotor.stop();
		
		DifferentialPilot pilot = new DifferentialPilot(2, 2, leftMotor, rightMotor);
		pilot.backward();
		
		Delay.msDelay(300);
		
		ultraSonicSensor.disable();
		
		pilot.stop();
		pilot.rotate(330);
		
		ultraSonicSensor.enable();
	}
	
	private static boolean isLeftTouched() {
		int sampleSize = rightTouchSensor.sampleSize();
		float[] samples = new float[sampleSize];
		rightTouchSensor.fetchSample(samples, 0);
		return samples[0] == 1;
	}

	private static float getDistanceToWall() {
		int sampleSize = ultraSonicSensor.sampleSize();
		float[] samples = new float[sampleSize];
		ultraSonicSensor.fetchSample(samples, 0);
		return samples[0];
	}
}


