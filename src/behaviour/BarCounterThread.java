package behaviour;

import javax.swing.colorchooser.ColorSelectionModel;

import javafx.scene.shape.Line;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import lejos.utility.Delay;

public class BarCounterThread extends Thread {

	private CheckIfSharpCornerThread checkerThread;
	final int gapBetweenLineAndBarCode = 1200;
	private MedianFilter filter;
	private LineBehaviour lineBehaviour;
	boolean firstLow = false;
	boolean isCountingBars = false;
	boolean barCodeComplete = false;
	float realTimeValue;
	long elapsedTime = 0;
	long timeOfBarCount = 0;
	long startTime = 0;
	boolean white = false;
	int counter = 0;
	int lowerBound = 350;
	private boolean isSharpCorner = false;
	boolean checkSharpCornerComplete = false;
	RegulatedMotor leftMotor;
	RegulatedMotor rightMotor;
	EV3ColorSensor colorSensor;

	public BarCounterThread(RegulatedMotor leftMotor, RegulatedMotor rightMotor, LineBehaviour line,
			EV3ColorSensor colorSensor) {
		this.lineBehaviour = line;
		this.colorSensor = colorSensor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		SampleProvider light = colorSensor.getMode("Red");
		filter = new MedianFilter(light, 4);
	}

	@Override
	public void run() {
		while (!this.isInterrupted()) {
			if (!isCountingBars) {
				checkIfBarCodeStarts();
			} else {
				countBarCode();
				checkIfBarCodeIsFinished();
				if (barCodeComplete) {
					stopDrivingForward();
					startBehaviorAccordingToBarCode();
				}
			}
			if (Button.readButtons() != 0) {
				stopFollowingLine();
				stopDrivingForward();
				this.interrupt();
			}
		}
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
				if (isSharpCorner) {
					keepFollowingLine();
				} else {
					stopFollowingLine();
					startBarCodeCounting();
					driveForwardToScanBarCode();
				}
			}
		}
	}
	
	private void checkIfBarCodeIsFinished() {
		if (realTimeValue < 0.12) {
			countElapsedTime(); 
		} else {
			firstLow = false;
		}
		if (elapsedTime > gapBetweenLineAndBarCode) {
			stopBarCodeCounting();
			barCodeComplete = true;
		}
	}

	public void countBarCode() {
		realTimeValue = getRealTimeValue();
		if (realTimeValue > 0.3) {
			white = true;
		} else if (realTimeValue < 0.1) {
			if (white) {
				white = false;
				counter++;
			}
		}
	}

	public void countElapsedTime() {
		if (!firstLow) {
			firstLow = true;
			startTime = System.currentTimeMillis();
		} else {
			elapsedTime = System.currentTimeMillis() - startTime;
		}
	}

	private void checkIfSharpLeftCorner() {
		lineBehaviour.stopFollowLine();
		checkerThread = new CheckIfSharpCornerThread(this, leftMotor, rightMotor, colorSensor);
		checkerThread.start();
	}

	private void keepFollowingLine() {
		elapsedTime = 0;
		lineBehaviour.keepFollowLine();
		lineBehaviour.passObstacle();
		isSharpCorner = false;
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

	private void startBehaviorAccordingToBarCode() {
		System.out.println("Bars" + counter);
		Button.waitForAnyPress();
		barCodeComplete = false;
		counter = 0;
		//test
		stopFollowingLine();
		stopDrivingForward();
		this.interrupt();
	}

	private void restartElapsedTimeCounting() {
		firstLow = false;
	}

	private void driveForwardToScanBarCode() {
		lineBehaviour.driveForwardToScanBarCode();
	}

	private void stopDrivingForward() {
		lineBehaviour.stopDrivingForward();
	}

	private void startBarCodeCounting() {
		firstLow = false;
		elapsedTime = 0;
		isCountingBars = true;
	}

	private void stopBarCodeCounting() {
		firstLow = false;
		elapsedTime = 0;
		isCountingBars = false;
	}

	private void stopFollowingLine() {
		lineBehaviour.stopFollowLine();
	}

	public float getRealTimeValue() {
		int samplesize = filter.sampleSize();
		float[] samples = new float[samplesize];
		filter.fetchSample(samples, 0);
		return (samples[0]*1.25f);
	}
}