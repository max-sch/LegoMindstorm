package behaviour;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import robot.RobotConfiguration;

public class LabyrinthBehaviour implements IBehaviour{

	private final RegulatedMotor leftMotor;
	private final RegulatedMotor rightMotor;
	private final EV3UltrasonicSensor ultraSonicSensor;
	private final EV3TouchSensor rightTouchSensor;

	private final static int baseSpeed = 650;
	private final static float minDistToWall = 0.05f;
	private final static float midDistToWall = 0.1f;
	private final static float maxDistToWall = 0.2f;

	public LabyrinthBehaviour(RobotConfiguration robotConf) {
		this.leftMotor = robotConf.getLeftMotor();
		this.rightMotor = robotConf.getRightMotor();
		this.ultraSonicSensor = robotConf.getUltraSonicSensor();
		this.rightTouchSensor = robotConf.getRightTouchSensor();
	}
	
	@Override
	public BarCode passObstacle() {
		float currentDistance;
		float turn;
		
		this.leftMotor.setSpeed(baseSpeed);
		this.rightMotor.setSpeed(baseSpeed);
		this.leftMotor.forward();
		this.rightMotor.forward();
		
		while (true) {
			currentDistance = getDistanceToWall();
			if (currentDistance <= minDistToWall) {
				turn = 100 - (5 - (currentDistance*100));
				onRev(baseSpeed, true);
				onRev((int) turn, false);
				
				this.leftMotor.forward();
				this.rightMotor.forward();
			} else if (currentDistance > minDistToWall && currentDistance <= midDistToWall) {
				onRev(baseSpeed, true);
				onRev(baseSpeed, false);
				
				this.leftMotor.forward();
				this.rightMotor.forward();
			} else if (currentDistance > midDistToWall && currentDistance < maxDistToWall) {
				turn = 100 - ((currentDistance*100) - 10);
				onRev((int) turn, true);
				onRev(baseSpeed, false);
				
				this.leftMotor.forward();
				this.rightMotor.forward();
			}
			else if (currentDistance > maxDistToWall) {
				onRev(300, true);
				onRev(baseSpeed, false);
				
				this.leftMotor.forward();
				this.rightMotor.forward();
			}
			
			if (isRightTouched()) {
				turnRoutine();
			}
			
			if (Button.readButtons() != 0) {
				break;
			}
		}
		
		//Has to be adapted
		return BarCode.FINISH;
	}
		
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
}
