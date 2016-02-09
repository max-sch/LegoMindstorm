package communication;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URL;
import java.net.URLConnection;
import java.util.Scanner;

public abstract class AbstractCommand<T> implements Command<T> {

	private static String CHARSET = "UTF-8";

	private final String name;
	protected final URL commandUrl;

	public AbstractCommand(URL commandUrl) {
		this.commandUrl = commandUrl;
		this.name = this.getClass().getSimpleName();
	}

	@Override
	public String getName() {
		return name;
	}

	protected String doGet() throws IOException {
		URLConnection connection = getAdjustedURL().openConnection();
		connection.setRequestProperty("Accept-Charset", CHARSET);
		InputStream response = connection.getInputStream();
		BufferedReader reader = new BufferedReader(new InputStreamReader((response)));

		String rspns = "";
		try (Scanner scanner = new Scanner(reader)) {
			while (scanner.hasNext())
				rspns += scanner.next();
		}

		return rspns;
	}
	
	// Override to add parameters
	protected URL getAdjustedURL() {
		return commandUrl;
	}

}
