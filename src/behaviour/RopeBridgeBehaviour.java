package behaviour;

import java.util.Iterator;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import robot.RobotConfiguration;

public class RopeBridgeBehaviour implements IBehaviour {

	private final RegulatedMotor rightMotor;
	private final RegulatedMotor leftMotor;
	private final EV3UltrasonicSensor ultraSonicSensor;
	private final EV3ColorSensor colorSensor;
	private final MedianFilter filter;
	private final SampleProvider provider;
	private final RegulatedMotor ultraSonicMotor;
	private final EV3TouchSensor leftTouchSensor;
	private final EV3TouchSensor rightTouchSensor;
	//private LineBehaviourThread thread;
	//private LabyrinthBehaviourThread labThread;
	
	public RopeBridgeBehaviour(RobotConfiguration robotConfig) {
		this.rightMotor = robotConfig.getRightMotor();
		this.leftMotor = robotConfig.getLeftMotor();
		this.ultraSonicSensor = robotConfig.getUltraSonicSensor();
		this.colorSensor = robotConfig.getColorSensor();
		this.provider = this.colorSensor.getRedMode();
		this.filter = new MedianFilter(provider, 10);
		this.ultraSonicMotor = robotConfig.getUltraSonicMotor();
		this.leftTouchSensor = robotConfig.getLeftTouchSensor();
		this.rightTouchSensor = robotConfig.getRightTouchSensor();
		//this.thread = new LineBehaviourThread(robotConfig);
		//this.labThread = new LabyrinthBehaviourThread(robotConfig);
	}
	
	@Override
	public BarCode passObstacle() {
		float currentDistance = getDistanceToWall();
		float realTimeValue, error, turn, powerA, powerB;
		int baseSpeed;
		float kp = 160; 
		float initialValue = 0.05f;	
		
		while (true) {
	
			currentDistance = getDistanceToWall();
			if (currentDistance < 0.3f) {
				break;
			}
			
			realTimeValue = getRealTimeValue();
			baseSpeed = 220;
			if(realTimeValue < 0.1) {
				error = (realTimeValue - initialValue)*20;
			} else if (realTimeValue > 0.4) {
				error = (realTimeValue - initialValue)*20;
			}
			else {
				error = (realTimeValue - initialValue)*7;
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
		
		this.leftMotor.stop();
		this.rightMotor.stop();
		
		
		rotateOptimal();
		
		ultraSonicMotor.rotate(-80);
		
		findEdge();
		
		Delay.msDelay(500);
		adaptedBridgeBehaviour();
		
		ultraSonicMotor.rotate(80);
		
		this.leftMotor.stop();
		this.rightMotor.stop();
		
//		this.leftMotor.setSpeed(0);
//		this.rightMotor.setSpeed(0);
//		
//		DifferentialPilot pilot = new DifferentialPilot(2, 10, leftMotor, rightMotor);
//		pilot.rotate(-30);
//		pilot.setTravelSpeed(250);
		
		
//		this.leftMotor.setSpeed(300);
//		this.rightMotor.setSpeed(0);
//		this.leftMotor.forward();
//		this.rightMotor.forward();
//		
//		this.leftMotor.stop();
//		this.rightMotor.stop();
//		
		//this.leftMotor.setSpeed(500);
		//this.rightMotor.setSpeed(500);
//		pilot.forward();
////		
//		Delay.msDelay(5000);
//		
//		currentDistance = getDistanceToWall();
//		while (currentDistance > 0.2f) {
//			currentDistance = getDistanceToWall();
//		}
//		
//		pilot.stop();
//		//this.labThread.start();
//		
//		
////		currentDistance = getDistanceToWall();
////		
////		while (currentDistance < 0.5f) {
////			currentDistance = getDistanceToWall();
////		}
////		
////		this.labThread.interrupt();
		
		return null;
	}
	
	private void rotateOptimal() {
		DifferentialPilot pilot = new DifferentialPilot(2, 10, leftMotor, rightMotor);
		float currentDistance;
		float threshold = 0.1f;
		
		for (int i = 0; i <= 50; i++) {
			pilot.rotate(-1);
			currentDistance = getDistanceToWall();
			if (currentDistance < threshold) {
				break;
			}
		}
		
	}
	
	private void findEdge() {
		float currentDistance;
		this.rightMotor.setSpeed(294);
		this.leftMotor.setSpeed(300);
		this.leftMotor.forward();
		this.rightMotor.forward();
		
		currentDistance = getDistanceToWall();
		//LCD.drawString("val: " + currentDistance, 0, 1);
		while (currentDistance < 0.2f) {
			currentDistance = getDistanceToWall();
			
			if (isRightTouched()) {
				this.leftMotor.stop();
				this.rightMotor.stop();
				new DifferentialPilot(2, 10, leftMotor, rightMotor).rotate(30);
				this.leftMotor.forward();
				this.rightMotor.forward();
			}
			
			if (Button.readButtons() != 0) {
				break;
			}
		}
	}
	
	private boolean isRightTouched() {
		int sampleSize = this.rightTouchSensor.sampleSize();
		float[] samples = new float[sampleSize];
		this.rightTouchSensor.fetchSample(samples, 0);
		return samples[0] == 1;
	}
	
	private float sanityCheck(float distance) {
		if (distance > 0.3f) {
			return 0.2f;
		}
		
		return distance;
	}
	
	private void adaptedBridgeBehaviour() {
		float currentDistance, secondDistance;
		float powerA, powerB, error, kp, turn, minDistance = 0.1f;
		int baseSpeed = 300;
		
		while (true) {
			currentDistance = getDistanceToWall();
			
			currentDistance = sanityCheck(currentDistance);
			
			Delay.msDelay(100);
			
			secondDistance = sanityCheck(getDistanceToWall());
			
			if ((currentDistance - secondDistance) > 0) {
				continue;
			}
			
			error = minDistance - currentDistance;
//			if (error < -0.30f) {
//				kp = 250;
			/*} else*/ if ( error < 0) {
				kp = 900; //800
			} else {
				kp = 350;//300
			}
			
			
			turn = kp * error;
			
			powerA = baseSpeed + turn;
			powerB = baseSpeed - turn;
			
			leftMotor.setSpeed((int) powerA);
			rightMotor.setSpeed((int) powerB);
			leftMotor.forward();
			rightMotor.forward();
			
			if (isLeftTouched()) {
				this.rightMotor.stop();
				this.leftMotor.stop();
			
				leftMotor.setSpeed(300);
				rightMotor.setSpeed(300);
				
				this.rightMotor.backward();
				this.leftMotor.backward();
				
				Delay.msDelay(1000);
				
				this.rightMotor.stop();
				this.leftMotor.stop();
				
				new DifferentialPilot(2, 10, leftMotor, rightMotor).rotate(-30);
			}
			
			if (isRightTouched()) {
				this.rightMotor.stop();
				this.leftMotor.stop();
			
				leftMotor.setSpeed(300);
				rightMotor.setSpeed(300);
				
				this.rightMotor.backward();
				this.leftMotor.backward();
				
				Delay.msDelay(1000);
				
				this.rightMotor.stop();
				this.leftMotor.stop();
				
				new DifferentialPilot(2, 10, leftMotor, rightMotor).rotate(30);
			}
			
			if (Button.readButtons() != 0) {
				break;
			}
		}
	}
	
	private boolean isLeftTouched() {
		int sampleSize = this.leftTouchSensor.sampleSize();
		float[] samples = new float[sampleSize];
		this.leftTouchSensor.fetchSample(samples, 0);
		return samples[0] == 1;
	}
	
	private void onRev(int Speed, boolean isRightMotor) {
		if (isRightMotor) {
			this.rightMotor.setSpeed(Speed);
		} else {
			this.leftMotor.setSpeed(Speed);
		}
	}
	
	private void rotateToOptimalPosition() {
		//float minValue;
		int counter = 25;
		//HashMap<Float, Integer> distanceValues = new HashMap();
		//DifferentialPilot pilot = new DifferentialPilot(2, 10, leftMotor, rightMotor);
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(0);
		
		leftMotor.forward();
		
		Delay.msDelay(2000);
		
//		for (int i = 0; i <= counter; i++) {
//			pilot.rotate(-1);
//			distanceValues.put(getDistanceToWall(),i);
//		}
//		
//		minValue = getMinValOf(distanceValues.keySet().iterator());
//		
//		pilot.rotate((counter) - distanceValues.get(minValue));
		//pilot.rotate(-counter);
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
	
	private float getRealTimeValue() {
		int samplesize = this.filter.sampleSize();
		float[] samples = new float[samplesize];
		this.filter.fetchSample(samples, 0);
		return samples[0];
	}

	private float getDistanceToWall() {
		int sampleSize = this.ultraSonicSensor.sampleSize();
		float[] samples = new float[sampleSize];
		ultraSonicSensor.fetchSample(samples, 0);
		return samples[0];
	}

}