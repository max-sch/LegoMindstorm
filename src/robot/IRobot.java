package robot;

import behaviour.BarCode;

public interface IRobot {
	
	public void passParkour();
	
	public void startRace();
	
	public void passObstacleWithBarCode(BarCode code);
}
