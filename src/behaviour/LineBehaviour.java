package behaviour;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import robot.RobotConfiguration;

public class LineBehaviour extends IBehaviour {

	EV3ColorSensor colorSensor;
	RegulatedMotor leftMotor;
	RegulatedMotor rightMotor;
	MedianFilter filter;
	private boolean followLine = true;
	private BarCounterThread barCounter;
	boolean isSharpCorner = false;
	
	private CheckIfSharpCornerThread checkerThread;
	final int gapBetweenLineAndBarCode = 1500;
	float realTimeValue;
	long elapsedTime = 0;
	boolean firstLow = false;
	long startTime = 0;
	
	public LineBehaviour(RobotConfiguration robotConfig) {
		this.rightMotor = robotConfig.getRightMotor();
		this.leftMotor = robotConfig.getLeftMotor();
		this.colorSensor = robotConfig.getColorSensor();
		SampleProvider light = colorSensor.getMode("Red");
		filter = new MedianFilter(light, 5);
	}	
	
	@Override
	public BarCode passObstacle() {
		colorSensor.setFloodlight(true);
		
		findLine();
		
		float realTimeValue;
		float kp = 160; 
		float initialValue = 0.05f;	
		float powerA = 0;
		float powerB = 0;
		float error = 0;
		float turn = 0;
		int baseSpeed = 125;

		//barCounter = new BarCounterThread(leftMotor, rightMotor, this, colorSensor);
		//barCounter.start();
		while (followLine) {
			realTimeValue = getRealTimeValue();
			baseSpeed = 220;
			if(realTimeValue < 0.1) {
				error = (realTimeValue - initialValue)*15;
			}
			else {
				error = (realTimeValue - initialValue)*5;
			}
			turn = kp * error;
			powerA = (baseSpeed + turn);
			powerB = (baseSpeed - turn);
			leftMotor.forward();
			rightMotor.forward();
			leftMotor.setSpeed((int) powerA);
			rightMotor.setSpeed((int) powerB);
		 	if (Button.readButtons() != 0) {
		 		barCounter.interrupt();
		 		Button.waitForAnyPress();
	    		break;
	    	}
		 	//checkIfBarCodeStarts();	 	
		}
		
		//barCounter.interrupt();
		
		rightMotor.stop();
		leftMotor.stop();
		
		// TODO has to be adapted
		return BarCode.FINISH;
	}
	
	public void driveForwardToScanBarCode() {
		leftMotor.setSpeed(220);
		rightMotor.setSpeed(220);
		leftMotor.forward();
		rightMotor.forward();
	}
	
	public void stopDrivingForward() {
		DifferentialPilot pilot = new DifferentialPilot(2, 10, leftMotor, rightMotor);
		pilot.stop();
	}
	
	public void stopFollowLine() {
		followLine = false;
	}
	
	public void keepFollowLine() {
		followLine = true;
	}
	
	public float getRealTimeValue() {
		int samplesize = filter.sampleSize();
		float[] samples = new float[samplesize];
		filter.fetchSample(samples, 0);
		return samples[0]*1.25f;
	}
	
	public void findLine() {
		LineScanner scanner = new LineScanner(this.colorSensor);
		scanner.start();
		
		DifferentialPilot pilot = new DifferentialPilot(2, 10, leftMotor, rightMotor);
		
		int counter = 0;
		int rotationAngle = 2;
		while(!lineFound) {
			pilot.rotate(rotationAngle);
			counter++;
			
			if (counter == 60) {
				rotationAngle = rotationAngle * (-1);
				counter = -60;
			}
		}
		
		pilot.stop();
	}
	
	
	private void checkIfBarCodeStarts() {
		realTimeValue = getRealTimeValue();
		if (realTimeValue < 0.1) {
			countElapsedTime();
		} else {
			restartElapsedTimeCounting();
		}
		if (elapsedTime >= gapBetweenLineAndBarCode) {
			checkIfSharpLeftCorner();
			synchronized (checkerThread) {
				try {
					checkerThread.wait();
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				if (!isSharpCorner) {
					followLine = false;
				}
			}
		}
	}
	
	private void checkIfSharpLeftCorner() {
		checkerThread = new CheckIfSharpCornerThread(this, leftMotor, rightMotor, colorSensor);
		checkerThread.start();
	}
	
	private void restartElapsedTimeCounting() {
		firstLow = false;
	}
	
	public void countElapsedTime() {
		if (!firstLow) {
			firstLow = true;
			startTime = System.currentTimeMillis();
		} else {
			elapsedTime = System.currentTimeMillis() - startTime;
		}
	}
	
	public void isSharpCorner() {
		firstLow = false;
		isSharpCorner = true;
		elapsedTime = 0;
	}

	public void setIsNoSharpCorner() {
		firstLow = false;
		isSharpCorner = false;
		elapsedTime = 0;
	}
}
