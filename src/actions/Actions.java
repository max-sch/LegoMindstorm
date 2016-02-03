package actions;

import lejos.robotics.RegulatedMotor;

public class Actions {

	final RegulatedMotor leftMotor;
	final RegulatedMotor rightMotor;

	public Actions(RegulatedMotor rightMotor, RegulatedMotor leftMotor) {
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
	}

	public void driveForward() {
		rightMotor.forward();
		leftMotor.forward();
	}

	public void driveBackward() {
		rightMotor.backward();
		leftMotor.backward();
	}

	public void turnLeftOnFoot() {
		rightMotor.forward();
		leftMotor.backward();
	}

	public void turnRightOnFoot() {
		rightMotor.backward();
		leftMotor.forward();
	}
	
	public void driveFast() {
		leftMotor.setSpeed(800);
		rightMotor.setSpeed(800);
	}

	public void driveNormalSpeed() {
		leftMotor.setSpeed(400);
		rightMotor.setSpeed(400);
	}
	
	public void driveSlow() {
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
	}
	
	public void stopMotors() {
		leftMotor.flt();
		rightMotor.flt();
	}
	
	public void stopWithBreaks() {
		leftMotor.stop();
		rightMotor.stop();
	}
}