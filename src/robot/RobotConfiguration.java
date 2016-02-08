package robot;

import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;


public class RobotConfiguration {

	private final RegulatedMotor leftMotor = Motor.C;
	private final RegulatedMotor rightMotor = Motor.B;
	private final RegulatedMotor ultraSonicMotor = Motor.A;
	private final EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	private final EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S3);
	private final EV3TouchSensor leftTouchSensor = new EV3TouchSensor(SensorPort.S4);
	private final EV3TouchSensor rightTouchSensor = new EV3TouchSensor(SensorPort.S1);
	
	public void initializeMotors() {
		this.leftMotor.resetTachoCount();
		this.rightMotor.resetTachoCount();
		this.leftMotor.rotateTo(0);
		this.rightMotor.rotateTo(0);
		this.leftMotor.setSpeed(400);
		this.rightMotor.setSpeed(400);
		this.leftMotor.setAcceleration(800);
		this.rightMotor.setAcceleration(800);
	}
	
	public RegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}
	
	public RegulatedMotor getRightMotor() {
		return this.rightMotor;
	}
	
	public RegulatedMotor getUltraSonicMotor() {
		return this.ultraSonicMotor;
	}
	
	public EV3UltrasonicSensor getUltraSonicSensor() {
		return this.ultrasonicSensor;
	}
	
	public EV3TouchSensor getLeftTouchSensor() {
		return this.leftTouchSensor;
	}
	
	public EV3TouchSensor getRightTouchSensor() {
		return this.rightTouchSensor;
	}
	
	public EV3ColorSensor getColorSensor() {
		return this.colorSensor;
	}
}
