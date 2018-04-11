package wifi;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;
import main.DriveManager;
import odometer.Display;
import odometer.OdometerExceptions;

/**
 * @author David Castonguay
 * 
 *         There are two variables you **MUST** set manually before trying to
 *         use this code.
 * 
 *         1. SERVER_IP: The IP address of the computer running the server
 *         application. This will be your own laptop, until the beta beta demo
 *         or competition where this is the TA or professor's laptop. In that
 *         case, set the IP to 192.168.2.3.
 * 
 *         2. TEAM_NUMBER: your project team number
 * 
 *         Note: We System.out.println() instead of LCD printing so that full
 *         debug output (e.g. the very long string containing the transmission)
 *         can be read on the screen OR a remote console such as the EV3Control
 *         program via Bluetooth or WiFi. You can disable printing from the WiFi
 *         code via ENABLE_DEBUG_WIFI_PRINT (below).
 *
 */
public class Wifi {

	// ** Set these as appropriate for your team and current situation **
	private static final String SERVER_IP = "192.168.2.11";
	private static final int TEAM_NUMBER = 12;

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	@SuppressWarnings("rawtypes")
	public void transmit() throws InterruptedException, OdometerExceptions {

		System.out.println("Running..");

		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data, catching any errors that might occur
		try {
			/*
			 * getData() will connect to the server and wait until the user/TA presses the
			 * "Start" button in the GUI on their laptop with the data filled in. Once it's
			 * waiting, you can kill it by pressing the upper left hand corner button
			 * (back/escape) on the EV3. getData() will throw exceptions if it can't connect
			 * to the server (e.g. wrong IP address, server not running on laptop, not
			 * connected to WiFi router, etc.). It will also throw an exception if it
			 * connects but receives corrupted data or a message from the server saying
			 * something went wrong. For example, if TEAM_NUMBER is set to 1 above but the
			 * server expects teams 17 and 5, this robot will receive a message saying an
			 * invalid team number was specified and getData() will throw an exception
			 * letting you know.
			 */
			Map data = conn.getData();

			DriveManager.RedTeam = ((Long) data.get("RedTeam")).intValue();
			DriveManager.RedCorner = ((Long) data.get("RedCorner")).intValue();
			DriveManager.GreenTeam = ((Long) data.get("GreenTeam")).intValue();
			DriveManager.GreenCorner = ((Long) data.get("GreenCorner")).intValue();

			DriveManager.OG = ((Long) data.get("OG")).intValue();
			DriveManager.OR = ((Long) data.get("OR")).intValue();

			DriveManager.Red_LLx = ((Long) data.get("Red_LL_x")).intValue();
			DriveManager.Red_LLy = ((Long) data.get("Red_LL_y")).intValue();
			DriveManager.Red_URx = ((Long) data.get("Red_UR_x")).intValue();
			DriveManager.Red_URy = ((Long) data.get("Red_UR_y")).intValue();

			DriveManager.Green_LLx = ((Long) data.get("Green_LL_x")).intValue();
			DriveManager.Green_LLy = ((Long) data.get("Green_LL_y")).intValue();
			DriveManager.Green_URx = ((Long) data.get("Green_UR_x")).intValue();
			DriveManager.Green_URy = ((Long) data.get("Green_LL_y")).intValue();

			DriveManager.TN_LLx = ((Long) data.get("TN_LL_x")).intValue();
			DriveManager.TN_LLy = ((Long) data.get("TN_LL_y")).intValue();
			DriveManager.TN_URx = ((Long) data.get("TN_UR_x")).intValue();
			DriveManager.TN_URy = ((Long) data.get("TN_UR_y")).intValue();

			DriveManager.BR_LLx = ((Long) data.get("BR_LL_x")).intValue();
			DriveManager.BR_LLy = ((Long) data.get("BR_LL_y")).intValue();
			DriveManager.BR_URx = ((Long) data.get("BR_UR_x")).intValue();
			DriveManager.BR_URy = ((Long) data.get("BR_UR_y")).intValue();

			DriveManager.SR_LLx = ((Long) data.get("SR_LL_x")).intValue();
			DriveManager.SR_LLy = ((Long) data.get("SR_LL_y")).intValue();
			DriveManager.SR_URx = ((Long) data.get("SR_UR_x")).intValue();
			DriveManager.SR_URy = ((Long) data.get("SR_UR_y")).intValue();

			DriveManager.SG_LLx = ((Long) data.get("SG_LL_x")).intValue();
			DriveManager.SG_LLy = ((Long) data.get("SG_LL_y")).intValue();
			DriveManager.SG_URx = ((Long) data.get("SG_UR_x")).intValue();
			DriveManager.SG_URy = ((Long) data.get("SG_UR_y")).intValue();

			// Example 1: Print out all received data
			System.out.println("Map:\n" + data);

			// Example 2 : Print out specific values

			// Example 3: Compare value

		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}

		System.out.print(
				"                                                                                                                                                 ");

		// if team red is 12, then TEAM is true, also means true for going to the bridge
		// first, otherwise we are team green (team is false)
		if (DriveManager.RedTeam == 12) {
			DriveManager.TEAM = true;
		} else {
			DriveManager.TEAM = false;

		}

		// Returns the proper constants that we are going to need.
		if (DriveManager.TEAM) {
			DriveManager.T12_FLAG = DriveManager.OR;
			DriveManager.T12_SC = DriveManager.RedCorner;
			DriveManager.T12_SLLx = DriveManager.SG_LLx;
			DriveManager.T12_SLLy = DriveManager.SG_LLy;
			DriveManager.T12_SURx = DriveManager.SG_URx;
			DriveManager.T12_SURy = DriveManager.SG_URy;

		} else {

			DriveManager.T12_FLAG = DriveManager.OG;
			DriveManager.T12_SC = DriveManager.GreenCorner;
			DriveManager.T12_SLLx = DriveManager.SR_LLx;
			DriveManager.T12_SLLy = DriveManager.SR_LLy;
			DriveManager.T12_SURx = DriveManager.SR_URx;
			DriveManager.T12_SURy = DriveManager.SR_URy;
		}
	}

}
