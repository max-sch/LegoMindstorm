package robot;

import java.util.HashMap;

import behaviour.BarCode;
import behaviour.IBehaviour;
import behaviour.LabyrinthBehaviour;
import behaviour.LineBehaviour;

public class Robot implements IRobot {
	
	private final RobotConfiguration robotConfig;
	private final HashMap<BarCode, IBehaviour> behaviourWithID;
	
	public Robot(RobotConfiguration robotConfig) {
		this.robotConfig = robotConfig;
		this.behaviourWithID = initBehaviour();
	}
	
	@SuppressWarnings("serial")
	private HashMap<BarCode, IBehaviour> initBehaviour() {
		// TODO has to be completed
		return new HashMap<BarCode, IBehaviour>() {{
			put(BarCode.LABYRINTH, new LabyrinthBehaviour(robotConfig));
			put(BarCode.LINE, new LineBehaviour(robotConfig));
		}};
	}

	@Override
	public void passParkour() {
		BarCode code = BarCode.LABYRINTH;
		
		while (!code.equals(BarCode.FINISH)) {
			code = this.behaviourWithID.get(code).passObstacle();
		}
		
		this.behaviourWithID.get(code).passObstacle();
	}

	@Override
	public void startRace() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void passObstacleWithBarCode(BarCode code) {
		this.behaviourWithID.get(code).passObstacle();
	}
}
