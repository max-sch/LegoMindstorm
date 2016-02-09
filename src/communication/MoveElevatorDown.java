package communication;

import java.io.IOException;
import java.net.URL;

public class MoveElevatorDown extends AbstractCommand<Boolean> {

	public MoveElevatorDown(URL commandUrl) {
		super(commandUrl);
	}

	@Override
	public Boolean execute() throws IOException {
		String response = doGet();
		return "OK".equalsIgnoreCase(response);
	}

}
