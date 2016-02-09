package behaviour;

import java.io.IOException;

import com.sun.prism.paint.Color;

import communication.ComModule;
import communication.Communication;
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
import robot.RobotConfiguration;

public class BridgeBehaviour implements IBehaviour {

	private final RegulatedMotor leftMotor;
	private final RegulatedMotor rightMotor;
	private final EV3UltrasonicSensor ultraSonicSensor;
	private final EV3ColorSensor colorSensor;
	private final RegulatedMotor ultraSonicMotor;
	private final EV3TouchSensor rightTouchSensor;
	private final EV3TouchSensor leftTouchSensor;
	private final MedianFilter filter;
	private SampleProvider provider;
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
		this.leftTouchSensor = robotConf.getLeftTouchSensor();
		this.provider = this.colorSensor.getRedMode();
		this.filter = new MedianFilter(this.provider, 3);
		
		this.ultraSonicMotor.rotate(0);
	}
	
	@Override
	public BarCode passObstacle() {
		ultraSonicMotor.rotate(-90);
		Delay.msDelay(500);
		
		findRightEdge();
		followRightEdge();
		
		driveToOptimalPosition();
		
//		try {
//			handleElevator();
//		} catch (IOException e) {
//			LCD.drawString("Error", 0, 1);
//		}
			
		this.ultraSonicMotor.rotate(90);
		
		//Has to be adapted
		return BarCode.FINISH;
	}
	
	private void handleElevator() throws IOException {
		ComModule com = Communication.getModule();
		DifferentialPilot pilot = new DifferentialPilot(2, 10, leftMotor, rightMotor);
		
		callElivator(com);
		
		waitUntilElevatorIsReady(com);
		
		Delay.msDelay(10000);
		
		driveIntoElevator(pilot); 
		
		pilot.backward();
		Delay.msDelay(200);
		pilot.stop();
		
		moveElevator(com);
		
		Delay.msDelay(5000);
		
		leaveElevator(pilot);
	}
	
	private void leaveElevator(DifferentialPilot pilot) {
		pilot.forward();
		
		Delay.msDelay(1500);
		
		pilot.stop();
	}

	private void moveElevator(ComModule com) throws IOException {
		while (true) {
			if (com.moveElevatorDown()) {
				break;
			}
		}
	}

	private void driveIntoElevator(DifferentialPilot pilot) {
		pilot.forward();
		
		while (!sensorsTouched()) {
			
			if (Button.readButtons() != 0) {
				break;
			}
		}
		
		pilot.stop();
	}
	
	private boolean sensorsTouched() {
		return (isRightTouched() && isLeftTouched());
	}
	
	private void waitUntilElevatorIsReady(ComModule com) throws IOException {
		while(true) {
			if (com.requestElevator()) {
				break;
			}
			
			if (Button.readButtons() != 0) {
				break;
			}
		}
	}
	
	private void callElivator(ComModule com) throws IOException {
		while (true) {
			if (com.requestStatus()) {
				break;
			}
			
			if (Button.readButtons() != 0) {
				break;
			}
		}
	}
	
	private void driveToOptimalPosition() {
//		this.leftMotor.stop();
//		this.rightMotor.stop();
		DifferentialPilot pilot = new DifferentialPilot(2, 10, leftMotor, rightMotor);
//		float currentDistance;
		pilot.stop();
		
//		for (int i = 0; i < 25; i++) {
//			pilot.rotate(-1);
//			currentDistance = getDistanceToGround();
//			if (currentDistance > 0.1f) {
//				break;
//			}
//		}
		
		pilot.rotate(40);
		pilot.forward();
		
		Delay.msDelay(1000);
		
		pilot.stop();
		pilot.rotate(-45);
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
		float rturn = 0;
		float kp;
		float rkp = 0;
		float error;
		float currentDistance;
		
		while (true) {
			
			float val = getColor();
			LCD.drawString("Val: " + val, 0, 1);
			if (val > 0.14f) {
				break;
			}
			
			currentDistance = getDistanceToGround();
			
			error = minDistance - currentDistance;
			
		    if ( error < 0) {
				kp = 800;
				rkp = 600;
			} else {
				kp = 200;//300
				rkp = kp;
			}
			
		    if (Math.abs(error) > 0.3f) {
		    	error = 0.2f;
		    }
			
			turn = kp * error;
			rturn = rkp * error;
			
			powerA = baseSpeed + turn;
			powerB = baseSpeed - rturn;
			
			leftMotor.setSpeed((int) powerA);
			rightMotor.setSpeed((int) powerB);
			leftMotor.forward();
			rightMotor.forward();
			
			if (Button.readButtons() != 0) {
				break;
			}
		}
	}

	private float getColor() {
		int sampleSize = this.filter.sampleSize();
		float[] samples = new float[sampleSize];
		this.filter.fetchSample(samples, 0);
		
		return samples[0];
	}

	private void findRightEdge() {
		this.rightMotor.setSpeed(293);
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
	
	private boolean isLeftTouched() {
		int sampleSize = this.leftTouchSensor.sampleSize();
		float[] samples = new float[sampleSize];
		this.leftTouchSensor.fetchSample(samples, 0);
		return samples[0] == 1;
	}
}
