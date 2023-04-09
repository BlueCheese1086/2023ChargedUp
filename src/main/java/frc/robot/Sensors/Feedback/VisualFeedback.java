package frc.robot.Sensors.Feedback;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisualFeedback extends SubsystemBase {
	private static final AddressableLED lights = new AddressableLED(Constants.LEDConstants.PORT);
	private static final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.COUNT);
	private static int startRainbowHue;
	private static int startDefaultIndex;
	private static double cheeseDex = 0.0;
	private static double cheeseMult = 1.0;

	private LEDMode mode = LEDMode.DefaultColor;

	private static VisualFeedback instance;

	public static VisualFeedback getInstance() {
		if (instance == null)
			instance = new VisualFeedback();
		return instance;
	}

	/** Creates a new Lights. */
	private VisualFeedback() {
		startRainbowHue = 0;
		startDefaultIndex = 0;

		lights.setLength(ledBuffer.getLength());
		lights.setData(ledBuffer);
		lights.start();
	}

	@Override
	public void periodic() {
		mode.execute();
	}

	private static void rainbow() {
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setHSV(i, (startRainbowHue + 2 * i) % 180, 200, 100);
		}
		lights.setData(ledBuffer);
		startRainbowHue = (startRainbowHue + 2) % 180;
	}

	private static void cone() {
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, 247, 227, 42);
		}
		lights.setData(ledBuffer);
	}

	private static void cube() {
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, 159, 24, 237);
		}
		lights.setData(ledBuffer);
	}

	private static void defaultColor() {
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			if ((i + startDefaultIndex) / 3 % 2 == 0) {
				ledBuffer.setRGB(i, 252, 236, 3);
			} else {
				ledBuffer.setRGB(i, 10, 10, 245);
			}
		}
		lights.setData(ledBuffer);
		startDefaultIndex++;
	}

	private static void bad() {
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, 250, 10, 10);
		}
		lights.setData(ledBuffer);
		startDefaultIndex++;
	}

	private static void good() {
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, 10, 250, 10);
		}
		lights.setData(ledBuffer);
		startDefaultIndex++;
	}

	private static void cheese() {
		if (cheeseDex > 1.0) {
			cheeseMult = -1;
			cheeseDex = 1;
		} else if (cheeseDex < 0.0) {
			cheeseMult = 1;
			cheeseDex = 0;
		}
		SmartDashboard.putNumber("/LEDS/DEX", cheeseDex);
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(
					i,
					(int) (Color.kYellow.red * cheeseDex * 255
							+ (1 - cheeseDex) * Color.kBlue.red * 255),
					(int) (Color.kYellow.green * cheeseDex * 255
							+ (1 - cheeseDex) * Color.kBlue.green * 255),
					(int) (Color.kYellow.blue * cheeseDex * 255
							+ (1 - cheeseDex) * Color.kBlue.blue * 255));
		}
		cheeseDex = cheeseDex + (1.0 / 250.0 * cheeseMult);
		// System.out.println(cheeseDex);
		lights.setData(ledBuffer);
	}

	public void setMode(LEDMode mode) {
		this.mode = mode;
	}

	public enum LEDMode {
		Rainbow(() -> rainbow()),
		Cone(() -> cone()),
		Cube(() -> cube()),
		Bad(() -> bad()),
		Good(() -> good()),
		Cheese(() -> cheese()),
		DefaultColor(() -> defaultColor());

		NullFunction method;

		LEDMode(NullFunction m) {
			method = m;
		}

		public void execute() {
			method.run();
		}
	}

	private interface NullFunction {
		void run();
	}
}