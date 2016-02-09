package communication;

import java.net.MalformedURLException;
import java.net.URL;

public final class Communication {

	private Communication() {
	}

	public static ComModule getModule() {
		try {
			URL baseUrl = new URL("http://192.168.0.5:5000");
			return new ComModule(baseUrl);
		} catch (MalformedURLException e) {
			throw new IllegalStateException("programming error, please revise hard coded URL");
		}
	}

}
