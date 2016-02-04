package behaviour;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;

public class Line2 {
	
	private RegulatedMotor rightMotor;
	private RegulatedMotor leftMotor;
	private EV3ColorSensor colorSensor;
	private float normalLight = 0.3f;
	private boolean onLine;
	private boolean direction = true;
	private float realTimeValue = 0;
	private float realTimeValueSum;
	private int times = 1;
	private int sumTimes = 0;
	
	public Line2(RegulatedMotor rightMotor, RegulatedMotor leftMotor, EV3ColorSensor colorSensor) {
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.colorSensor = colorSensor;
	}
	
//	public void calibrate() {
//		float sum = 0.0f;
//		for (int i = 0; i < 1000; i++) {
//			sum += getRealTimeValue();
//		}
//		normalLight = (sum / 1000.0f) + 0.1f;
//		LCD.drawString("Calib: " + normalLight, 0, 1);
//		while (true) {
//			if (Button.readButtons() != 0) {
//				break;
//			}
//		}
//	}
	
	public void follow() {
		while (Button.readButtons() == 0) {
			onLine = isOnLine();
			if (onLine) {
				realTimeValue = getRealTimeValue();
				if (realTimeValue > 0.4) {
					setPower(300);
				} else if (realTimeValue > 0.5) {
					setPower(400);
				} else {
					setPower(200);
				}
				leftMotor.forward();
				rightMotor.forward();
			} else if (!onLine) {
//				realTimeValueSum = 0;
//				times = 1;
//				sumTimes = 0;
				rightMotor.stop();
				leftMotor.stop();
				setMotorDirection();
				rotateTime(1000);
				if (!onLine) {
					reverseMotor();
					rotateUntilOnLine();
				}
			}
			if (Button.readButtons() != 0) {
				break;
			}
		}
	}
	
	//rotiert eine bestimmte Zeit und überprüft, ob er wieder auf der Linie ist.
	private void rotateTime(int time) {
		int t =(int) System.currentTimeMillis() + time;
		setPower(50);
		while ((int)System.currentTimeMillis() < t && !isOnLine() && Button.readButtons()== 0) {
//			realTimeValueSum = 0;
//			times = 1;
//			sumTimes = 0;
		}
		onLine = isOnLine();
	}
	
	//rotiert bis er wieder auf der Linie ist
	private void rotateUntilOnLine() {
		setPower(50);
		while (!isOnLine() && Button.readButtons()== 0) {
//			realTimeValueSum = 0;
//			times = 1;
//			sumTimes = 0;
		}
		onLine = true;
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
	
	//gibt den akutell gemessenen Lichtwert zurück
	private float getRealTimeValue() {
		int samplesize = colorSensor.sampleSize();
		float[] samples = new float[samplesize];
		colorSensor.getRedMode().fetchSample(samples, 0);
		return samples[0];
	}
	
	//Gibt aus, ob sich der Roboter auf der Linie befindet, indem er den
	//aktuell gemessenen Wert(getRealTimeValue()) mit dem Wert, den der noramle Boden reflektiert(normalLight)
	private boolean isOnLine() {
		return (getRealTimeValue() > normalLight);
	}
	
	private boolean isOnLine2() {
		realTimeValueSum += (times * getRealTimeValue());
		sumTimes += times;
		times++;
		realTimeValue = realTimeValueSum / sumTimes;
		return (realTimeValue > normalLight);
	}
	
	private void setPower(int p) {
		leftMotor.setSpeed(p);
		rightMotor.setSpeed(p);
	}
	
	//Thread schlafen legen für aMilliseconds
	private void sleep(int aMilliseconds) {
		try {
			Thread.sleep(aMilliseconds);
		} catch (Exception e) {
			
		}
	}
}
