package behaviour;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.PilotProps;
import robot.Robot;

public class Labyrinth {
	
	private final static float wallDistance = 0.05f;
	
	private final RegulatedMotor leftMotor;
	private final RegulatedMotor rightMotor;
	private final EV3UltrasonicSensor ultraSonicSensor;
	
	public Labyrinth(Robot robotConf) {
		this.leftMotor = robotConf.getLeftMotor();
		this.rightMotor = robotConf.getRightMotor();
		this.ultraSonicSensor = robotConf.getUltraSonicSensor();
		
	}
	
	public void explore() {
		while (true) {
//			
//			adjustWallDistance(wallDistance > getDistanceToWall());
//			
//			if (Button.readButtons() != 0) {
//	    		break;
//	    	}
//		}

		//float currentDistance = getDistanceToWall();
		DifferentialPilot pilot = new DifferentialPilot(2, 2, this.leftMotor, this.rightMotor);
		if (rotateUntilShortestDistanceFound()) {
			pilot.setTravelSpeed(200);
			pilot.forward();
		}
		
		if (Button.readButtons() != 0) {
			break;
		}
		
		}
		
		
	}
	
	private boolean rotateUntilShortestDistanceFound() {
		//float currentDistance;
		float shortestDistance = getDistanceToWall();
		
		DifferentialPilot pilot = new DifferentialPilot(2, 2, this.leftMotor, this.rightMotor);
		pilot.setRotateSpeed(50);
		pilot.rotateLeft();
		
		while (getDistanceToWall() < shortestDistance) {
			shortestDistance = getDistanceToWall();
		}
		pilot.stop();
		pilot.rotateRight();
		while (getDistanceToWall() < shortestDistance) {
			shortestDistance = getDistanceToWall();
		}
		pilot.stop();
		
		
		return true;
		
		
	}
	
//	private void adjustWallDistance(boolean isNearWall) {
//		if (isNearWall) {
//			driveAwayFromWall();
//			return;
//		} 
//		
//		driveToWall();
//	}
//	
//	private void driveToWall() {
//		float currentDistance;
//		RegulatedMotor leftMotor = robotConf.getLeftMotor();
//		RegulatedMotor rightMotor = robotConf.getRightMotor();
//		
//		currentDistance = getDistanceToWall();
//		
//		DifferentialPilot pilot = new DifferentialPilot(2, 2, leftMotor, rightMotor);
//		pilot.rotate(60);
//		leftMotor.setSpeed(300);
//		rightMotor.setSpeed(300);
//		pilot.forward();
//		
//		while (currentDistance >= 0.1f) {
//			currentDistance = getDistanceToWall();
//		}
//		
//		pilot.stop();
//		pilot.rotate(-60);
//	}
//	
//	private void driveAwayFromWall() {
//		
//	}
//
	private float getDistanceToWall() {
		int sampleSize = this.ultraSonicSensor.sampleSize();
		float[] samples = new float[sampleSize];		
		
		return samples[0];
	}
	
}
