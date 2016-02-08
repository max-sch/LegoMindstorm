package behaviour;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import robot.RobotConfiguration;

public class RopeBridgeBehaviour implements IBehaviour {

	private final RegulatedMotor rightMotor;
	private final RegulatedMotor leftMotor;
	private final EV3UltrasonicSensor ultraSonicSensor;
	private LineBehaviourThread thread;
	//private LabyrinthBehaviourThread labThread;
	
	public RopeBridgeBehaviour(RobotConfiguration robotConfig) {
		this.rightMotor = robotConfig.getRightMotor();
		this.leftMotor = robotConfig.getLeftMotor();
		this.ultraSonicSensor = robotConfig.getUltraSonicSensor();
		this.thread = new LineBehaviourThread(robotConfig);
		//this.labThread = new LabyrinthBehaviourThread(robotConfig);
	}
	
	@Override
	public BarCode passObstacle() {
		float currentDistance = getDistanceToWall();
		
		thread.start();
		
		while (currentDistance > 0.2f) {
	
			currentDistance = getDistanceToWall();
			
			if (Button.readButtons() != 0) {
				break;
			}
		}
		
		thread.setStopped(true);
		
		rotateToOptimalPosition();
		//this.labThread.start();
		
		
//		currentDistance = getDistanceToWall();
//		
//		while (currentDistance < 0.5f) {
//			currentDistance = getDistanceToWall();
//		}
//		
//		this.labThread.interrupt();
		
		return null;
	}
	
	private void rotateToOptimalPosition() {
		float minValue;
		int counter = 25;
		HashMap<Float, Integer> distanceValues = new HashMap();
		DifferentialPilot pilot = new DifferentialPilot(2, 10, leftMotor, rightMotor);
		
		for (int i = 0; i <= counter; i++) {
			pilot.rotate(-i);
			distanceValues.put(getDistanceToWall(),i);
		}
		
		minValue = getMinValOf(distanceValues.keySet().iterator());
		
		pilot.rotate((counter) - distanceValues.get(minValue));
	}
	
	private float getMinValOf(Iterator<Float> distanceValues) {
		
		float currentMin = distanceValues.next();
		float currentVal;
		
		while (distanceValues.hasNext()) {
			currentVal = distanceValues.next();
			
			if (currentMin > currentVal) {
				currentMin = currentVal;
			}
		}
		
		return currentMin;
	}

	private float getDistanceToWall() {
		int sampleSize = this.ultraSonicSensor.sampleSize();
		float[] samples = new float[sampleSize];
		ultraSonicSensor.fetchSample(samples, 0);
		return samples[0];
	}

}