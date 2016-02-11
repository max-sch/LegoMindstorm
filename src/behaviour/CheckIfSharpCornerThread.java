package behaviour;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.internal.io.SystemSettings;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
import lejos.robotics.navigation.DifferentialPilot;

public class CheckIfSharpCornerThread extends Thread {

	private MedianFilter filter;
	boolean isSharpCorner = false;
	RegulatedMotor leftMotor;
	RegulatedMotor rightMotor;
	LineBehaviour line;
	EV3ColorSensor colorSensor;

	public CheckIfSharpCornerThread(LineBehaviour line, RegulatedMotor leftMotor, RegulatedMotor rightMotor,
			EV3ColorSensor colorSensor) {
		this.line = line;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.colorSensor = colorSensor;
		SampleProvider light = colorSensor.getMode("Red");
		filter = new MedianFilter(light, 2);
	}

	@Override
	public void run() {
		synchronized (this) {
			checkIfSharpCorner();
			notify();
		}
	}

	public void checkIfSharpCorner() {
		leftMotor.stop();
		rightMotor.stop();
		isSharpCorner = false;
		int performedRotation = 0;
		DifferentialPilot pilot = new DifferentialPilot(2, 10, leftMotor, rightMotor);
		if (getRealTimeValue() >= 0.1) {
			pilot.rotate(-4);
			isSharpCorner = true;
			tellLineBehaviorIsSharpCorner();
		} else {
			for (int i = 0; i <= 40; i++) {
				pilot.rotate(1);
				performedRotation += 1;
				if (getRealTimeValue() >= 0.1) {
					isSharpCorner = true;
					tellLineBehaviorIsSharpCorner();
					break;
				}
				if (Button.readButtons() != 0) {
					this.interrupt();
				}
			}
		}
		if (!isSharpCorner) {
			align(pilot, performedRotation);
			line.setIsNoSharpCorner();
		}
	}

	private void tellLineBehaviorIsSharpCorner() {
		line.isSharpCorner();
	}

	private void align(DifferentialPilot pilot, int performedRotation) {
		pilot.rotate(-performedRotation);
		pilot.rotate(-45);
	}

	public float getRealTimeValue() {
//		int samplesize = filter.sampleSize();
//		float[] samples = new float[samplesize];
		
//		System.out.println("Value:"+counter.getRealTimeValue());
//		Button.waitForAnyPress();
//		System.out.flush();
		return line.getRealTimeValue();
//		return samples[0];
	}
}
