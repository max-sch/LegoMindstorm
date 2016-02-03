package behaviour;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import robot.Robot;

public class Labyrinth {
	
	private final static float wallDistance = 0.05f;
			
	private final Robot robotConf;
	
	public Labyrinth(Robot robotConf) {
		this.robotConf = robotConf;
	}
	
	public void explore() {
		while (true) {
			
			adjustWallDistance(wallDistance > getDistanceToWall());
			
			if (Button.readButtons() != 0) {
	    		break;
	    	}
		}
	}
	
	private void adjustWallDistance(boolean isNearWall) {
		if (isNearWall) {
			driveAwayFromWall();
		} 
		
		driveToWall();
	}
	
	private void driveToWall() {
		float currentDistance;
		RegulatedMotor leftMotor = robotConf.getLeftMotor();
		RegulatedMotor rightMotor = robotConf.getRightMotor();
		
		currentDistance = getDistanceToWall();
		
		DifferentialPilot pilot = new DifferentialPilot(2, 2, leftMotor, rightMotor);
		pilot.rotate(60);
		leftMotor.setSpeed(300);
		rightMotor.setSpeed(300);
		pilot.forward();
		
		while (currentDistance >= 0.1f) {

		}
		
		pilot.stop();
		pilot.rotate(-60);
	}
	
	private void driveAwayFromWall() {
		
	}

	private float getDistanceToWall() {
		EV3UltrasonicSensor ultraSonicSensor = robotConf.getUltraSonicSensor();
		int sampleSize = ultraSonicSensor.sampleSize();
		float[] samples = new float[sampleSize];		
		
		return samples[0];
	}
	
}
