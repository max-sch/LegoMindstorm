package behaviour;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import robot.RobotConfiguration;

public class RollingFieldBehaviour implements IBehaviour {

	private final RegulatedMotor rightMotor;
	private final RegulatedMotor leftMotor;
	private final EV3UltrasonicSensor ultraSonicSensor;
	private final EV3ColorSensor colorSensor;
	
	private final static int baseSpeed = 400;
	private final static float minDistToWall = 0.05f;
	private final static float midDistToWall = 0.1f;
	private final static float maxDistToWall = 0.2f;
	
	public RollingFieldBehaviour(RobotConfiguration robotConfig) {
		this.rightMotor = robotConfig.getRightMotor();
		this.leftMotor = robotConfig.getLeftMotor();
		this.ultraSonicSensor = robotConfig.getUltraSonicSensor();
		this.colorSensor = robotConfig.getColorSensor();
	}
	
	@Override
	public BarCode passObstacle() {
		
		float currentDistance, turn;
		
		this.rightMotor.setSpeed(300);
		this.leftMotor.setSpeed(300);
		
		this.rightMotor.forward();
		this.leftMotor.forward();
		
		Delay.msDelay(1000);
		
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
			
			if (Button.readButtons() != 0) {
				break;
			}
		
		}
		
		return BarCode.LINE;
	}
		
	private void onRev(int Speed, boolean isRightMotor) {
		if (isRightMotor) {
			this.rightMotor.setSpeed(Speed);
		} else {
			this.leftMotor.setSpeed(Speed);
		}
	}	
		
	private float getDistanceToWall() {
		int sampleSize = this.ultraSonicSensor.sampleSize();
		float[] samples = new float[sampleSize];
		ultraSonicSensor.fetchSample(samples, 0);
		return samples[0];
	}

}
