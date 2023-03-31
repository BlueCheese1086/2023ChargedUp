package frc.robot.StateManager;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Arm.ArmState;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Configuration.ControllableConfiguration;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Sensors.Feedback.VisualFeedback.LEDMode;
import frc.robot.Sensors.Field.FieldElements;
import frc.robot.Wrist.WristState;
import frc.robot.Wrist.WristSubsystem;

public class StateManager extends SubsystemBase {

	private static StateManager instance;

	private ArmSubsystem arm;
	private WristSubsystem wrist;

	private final double ARM_LENGTH = ArmConstants.ARM_LENGTH;
	private final double INTAKE_LENGTH = IntakeConstants.INTAKE_LENGTH;

	private double elevatorHeight = 0;
	private double armAngle = 0;
	private double wristAngle = 0;

	private Positions desiredPosition = Positions.stowed;
	private Positions currentPosition = Positions.stowed;

	private final HashMap<String, ControllableConfiguration> configurations = new HashMap<>();

	private Mechanism2d totalMech = new Mechanism2d(2, 2);
	MechanismRoot2d root = totalMech.getRoot("elevator", 0, 0);
	MechanismLigament2d evLigament = root.append(new MechanismLigament2d("ev", elevatorHeight,
			Units.radiansToDegrees(Math.PI / 2), 2, new Color8Bit(255, 0, 0)));
	MechanismLigament2d armLigament = evLigament.append(
			new MechanismLigament2d("arm", ARM_LENGTH, 90.0));
	MechanismLigament2d wristLigament = armLigament.append(
		new MechanismLigament2d("wrist", INTAKE_LENGTH, 90.0)
	);

	public static StateManager getInstance() {
		return instance;
	}

	public StateManager(ArmSubsystem arm, WristSubsystem w) {
		instance = this;
		this.arm = arm;
		this.wrist = w;
		Shuffleboard.getTab("Field").add("Mech", totalMech);

		Shuffleboard.getTab("State").addDouble("Arm", () -> arm.getCurrentState().angle);
		Shuffleboard.getTab("State").addDouble("Wrist", () -> wrist.getCurrentState().angle);

		configurations.putAll(Map.ofEntries(
				Map.entry("PieceMode", new ControllableConfiguration("State Manager", "Piece Mode", Piece.Cube)),
                Map.entry("Enabled", new ControllableConfiguration("Subsystems", "State Manager Enabled", true))
        ));
	}

	@Override
	public void periodic() {
		armAngle = arm.getCurrentState().angle;
		wristAngle = wrist.getCurrentState().angle;

		armLigament.setAngle(new Rotation2d(armAngle - Units.degreesToRadians(evLigament.getAngle())));
		wristLigament.setAngle(new Rotation2d(wristAngle));
	}

	public ArmState getArmState() {
		return arm.getCurrentState();
	}

	public WristState getWristState() {
		return wrist.getCurrentState();
	}

	public void setPosition(Positions p) {
		desiredPosition = p;
		currentPosition = p;
	}

	public void executePosition() {
		Positions current = this.getCurrentPosition();
		Positions desired = this.getDesiredPosition();

		double[] vals = desired.getEffectors();
		arm.setAngle(vals[1]);
		// wrist.setAngle(((Piece) configurations.get("piecemode")).pickupAngle, false);
		System.out.println(vals[2]);
		if (vals[2] != -100) {
			wrist.setAngle(vals[2], false);
		} else {
			wrist.setAngle(((Piece) configurations.get("PieceMode").getValue()).getWristAngle(), false);
		}

		if (current != desired && current.isHere(vals)) {
			this.setPosition(desired);
		}
	}

	public Positions getCurrentPosition() {
		return currentPosition;
	}

	public Positions getDesiredPosition() {
		return desiredPosition;
	}

	public Object getConfiguration(String key) {
		return configurations.get(key);
	}

	public Piece getPieceMode() {
		return (Piece) configurations.get("PieceMode").getValue();
	}

	public void setPieceMode(Piece p) {
        configurations.get("PieceMode").setValue(p);
	}

	public enum Piece {
		Cube(LEDMode.Cube, 0, -1),
		Cone(LEDMode.Cone, -Math.PI/4, -1);

		private LEDMode mode;
		private double pickupAngle;
		private double intakeSpeed;

		private Piece(LEDMode mode, double pickupAngle, double intakeSpeed) {
			this.mode = mode;
			this.pickupAngle = pickupAngle;
			this.intakeSpeed = intakeSpeed;
		}

		public LEDMode getLEDMode() {
			return mode;
		}

		public double getWristAngle() {
			return pickupAngle;
		}

		public double getIntakeSpeed() {
			return intakeSpeed;
		}
	}

	public static enum Positions {
		stowed(1, -1.0),
		transition(1, -1.0),
		ground(1, FieldElements.hybridStations.get(1).getZ()),
		mid(2, FieldElements.hybridStations.get(1).getZ() + FieldConstants.SCORING_DISTANCE_Z * 1),
		high(3, FieldElements.hybridStations.get(1).getZ() + FieldConstants.SCORING_DISTANCE_Z * 2),
		player(3, FieldElements.hybridStations.get(1).getZ() + FieldConstants.SCORING_DISTANCE_Z * 2);

		boolean goingIn = false;

		double tolerance = 0.1;

		int index;

		double height;

		Positions(int a, double height) {
			index = a;
			this.height = height;
		}

		public double[] getEffectors() {
			switch (this) {
				case stowed:
					return new double[] { 0.05, ArmConstants.UPPER_RANGE, Math.PI/2
					 };
				case transition:
					return new double[] {-1.3, Math.PI/4};
				case ground:
					return new double[] {-0.9, -100};
				case mid:
					return new double[] {Math.PI/6, -100};
				case high:
					return new double[] {Math.PI/4, -100};
				case player:
					return new double[] {Math.PI/4, -100};
			}
			return new double[] {};
		}

		public boolean isHere(double[] values) {
			if (values[2] != -100)
				return ((Math.abs(values[0] - getEffectors()[0]) < tolerance) &&
						(Math.abs(values[1] - getEffectors()[1]) < tolerance) &&
						(Math.abs(values[2] - getEffectors()[2]) < tolerance));
			return ((Math.abs(values[0] - getEffectors()[0]) < tolerance) &&
					(Math.abs(values[1] - getEffectors()[1]) < tolerance));
		}

		public double getHeight() {
			return height;
		}

		public int getMultiplier() {
			return index;
		}
	}
}