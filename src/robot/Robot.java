package robot;

import java.util.HashMap;

import behaviour.BarCode;
import behaviour.BridgeBehaviour;
import behaviour.IBehaviour;
import behaviour.LabyrinthBehaviour;
import behaviour.LineBehaviour;
import behaviour.RopeBridgeBehaviour;

public class Robot implements IRobot {
	
	private final RobotConfiguration robotConfig;
	private final  HashMap<BarCode, IBehaviour> behaviourWithID;
	
	public Robot(RobotConfiguration robotConfig) {
		this.robotConfig = robotConfig;
		this.behaviourWithID = new HashMap<>();
		initBehaviour();
	}
	
	private void initBehaviour() {
		// TODO has to be completed
		this.behaviourWithID.put(BarCode.LABYRINTH, new LabyrinthBehaviour(robotConfig));
		this.behaviourWithID.put(BarCode.LINE, new LineBehaviour(robotConfig));
		this.behaviourWithID.put(BarCode.BRIDGE, new BridgeBehaviour(robotConfig));
		this.behaviourWithID.put(BarCode.ROPEBRIDGE, new RopeBridgeBehaviour(robotConfig));
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
