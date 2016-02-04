package behaviour;
import lejos.hardware.Button;

import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;

public class Line3 {
	private RegulatedMotor rightMotor;
	private RegulatedMotor leftMotor;
	private EV3ColorSensor colorSensor;
	private float normalLight = 0.2f;
	private boolean onLine;
	private boolean direction = true;
	private float realTimeCValue = 0;
	private float oldRealTimeValue;
	DifferentialPilot pilot;
	
	public Line3(RegulatedMotor rightMotor, RegulatedMotor leftMotor, EV3ColorSensor colorSensor) {
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.colorSensor = colorSensor;
		pilot = new DifferentialPilot(2,10, leftMotor, rightMotor);
	}
	
	public void searchLine() {
		while (true ) {
			pilot.forward();
			pilot.setTravelSpeed(2);
			sleep(1000);
			onLine = isOnLine();
			if (!onLine) {
				onLine = rotateAndReturnTrueIfLineFound(70, 3);
				onLine = rotateAndReturnTrueIfLineFound(-70, -3);
			}
			 else {
				pilot.stop();
				oldRealTimeValue = getRealTimeValue();
				setLineStartingPosition();
				break;
			}
			if (Button.readButtons() != 0) {
				break;
			}
		}
	}
	
	public void setLineStartingPosition() {
		boolean goodStartingPosition = false;
		int deg = 5;
		while (!goodStartingPosition) {
			pilot.rotate(deg);
			if (getRealTimeValue() < oldRealTimeValue) {
				if (getRealTimeValue() > 0.15) {
					goodStartingPosition = true;
				} else {
					deg =-2;
				}
			} else {
				deg = 2;
			}
		}
	}
	
	
	
	public void follow() {
		while (true) {
			onLine = isOnLine();
//			oldRealTimeValue = realTimeValue;
//			realTimeValue = getRealTimeValue();
			if (onLine) {
				leftMotor.forward();
				rightMotor.forward();
				setPower(200);
				while (isOnLine());
			} else {
				leftMotor.stop();
				rightMotor.stop();
				int rotateTime = 200;
				reverseMotor();
				setMotorDirection();
				while (!isOnLine()) {
					rotateTime(rotateTime);
					if (isOnLine()) {
						break;
					}
					rotateTime += 100;
					reverseMotor();
				}
			}
			if (Button.readButtons() != 0) {
				break;
			}
		}
	}
	
	private void rotateTime(int time) {
		int t =(int) System.currentTimeMillis() + time;
		setMotorDirection();
		setPower(100);
		while (!isOnLine() && (int)System.currentTimeMillis() < t);
		onLine = isOnLine();
	}
	
	private void setMotorDirection() {
		if (direction) {
			leftMotor.forward();
			rightMotor.backward();
		} else {
			leftMotor.backward();
			rightMotor.forward();
		}
	}
	
	private void reverseMotor() {
		direction = !direction;
		leftMotor.flt();
		rightMotor.flt();
		sleep(100);
		if (direction) {
			leftMotor.forward();
			rightMotor.backward();
		} else {
			leftMotor.backward();
			rightMotor.forward();
		}
	}
	
	private boolean isOnLine() {
		return (getRealTimeValue() > normalLight);
	}
	
	private float getRealTimeValue() {
		int samplesize = colorSensor.sampleSize();
		float[] samples = new float[samplesize];
		colorSensor.getRedMode().fetchSample(samples, 0);
		return samples[0];
	}
	
	private void setPower(int p) {
		leftMotor.setSpeed(p);
		rightMotor.setSpeed(p);
	}
	
	public boolean rotateAndReturnTrueIfLineFound(int totalRotation, int rotationStep) {
		int executedRotation = 0;
		while (executedRotation < totalRotation) {
			pilot.rotate(rotationStep);
			executedRotation += rotationStep;
			if (isOnLine()) {
				pilot.stop();
				oldRealTimeValue = getRealTimeValue();
				setLineStartingPosition();
				return true;
			}
		}
		pilot.rotate(-totalRotation);
		return false;
	}	
	
	private void sleep(int aMilliseconds) {
		try {
			Thread.sleep(aMilliseconds);
		} catch (Exception e) {
			
		}
	}
}
