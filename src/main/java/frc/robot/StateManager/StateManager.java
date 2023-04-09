package frc.robot.StateManager;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Arm.ArmState;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Configuration.ControllableConfiguration;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Sensors.Feedback.VisualFeedback;
import frc.robot.Sensors.Feedback.VisualFeedback.LEDMode;
import frc.robot.Sensors.Field.FieldElements;
import frc.robot.Sensors.Field.PositionManager;
import frc.robot.Wrist.WristState;
import frc.robot.Wrist.WristSubsystem;

public class StateManager extends SubsystemBase {

	private static StateManager instance;

	private ArmSubsystem arm;
	private WristSubsystem wrist;
	private IntakeSubsystem intake;
	private VisualFeedback leds;

	private final double ARM_LENGTH = ArmConstants.ARM_LENGTH;
	private final double INTAKE_LENGTH = IntakeConstants.INTAKE_LENGTH;

	private double armAngle = 0;
	private double wristAngle = 0;

	private Positions desiredPosition = Positions.stowed;
	private Positions currentPosition = Positions.stowed;

	private Piece pieceMode;

	private final HashMap<String, ControllableConfiguration> configurations = new HashMap<>();

	private Mechanism2d totalMech = new Mechanism2d(2, 2);
	MechanismRoot2d root = totalMech.getRoot("elevator", 0, 0);
	MechanismLigament2d evLigament = root.append(new MechanismLigament2d(
		"ev", 
		Units.inchesToMeters(32),
		81, 
		2, 
		new Color8Bit(255, 0, 0)
	));
	MechanismLigament2d armLigament = evLigament.append(new MechanismLigament2d(
		"arm", 
		ARM_LENGTH, 
		90.0
	));
	MechanismLigament2d wristLigament = armLigament.append(new MechanismLigament2d(
		"wrist", 
		INTAKE_LENGTH, 
		90.0
	));

	public static StateManager getInstance() {
		return instance;
	}

	public StateManager(ArmSubsystem arm, WristSubsystem w, IntakeSubsystem i) {
		instance = this;
		this.arm = arm;
		this.wrist = w;
		this.intake = i;
		this.leds = VisualFeedback.getInstance();
		Shuffleboard.getTab("Field").add("Mech", totalMech);

		Shuffleboard.getTab("State").addDouble("Arm", () -> arm.getCurrentState().angle);
		Shuffleboard.getTab("State").addDouble("Wrist", () -> wrist.getCurrentState().angle);

		configurations.putAll(Map.ofEntries(
				Map.entry("PieceMode", new ControllableConfiguration("State Manager", "Piece Mode", Piece.Cube)),
                Map.entry("Enabled", new ControllableConfiguration("Subsystems", "State Manager Enabled", true))
        ));

		pieceMode = (Piece) configurations.get("PieceMode").getValue();
	}

	@Override
	public void periodic() {
		armAngle = arm.getCurrentState().angle;
		wristAngle = wrist.getCurrentState().angle;

		armLigament.setAngle(new Rotation2d(armAngle - Units.degreesToRadians(evLigament.getAngle())));
		wristLigament.setAngle(new Rotation2d(wristAngle - armAngle));

		pieceMode = (Piece) configurations.get("PieceMode").getValue();

		if (DriverStation.isEnabled() && VisualFeedback.getInstance().getCurrentCommand() == null) {
			leds.setMode(pieceMode == Piece.Cone ? LEDMode.Cone : LEDMode.Cube);
		}

		if (pieceMode == Piece.Cone) {
			PositionManager.getInstance().getObject("Cone").setPose(
				new Pose2d(10.0, 5.0, new Rotation2d())
			);
			PositionManager.getInstance().getObject("Cube").setPose(
				new Pose2d(-100, -100, new Rotation2d())
			);
		} else if (pieceMode == Piece.Cube) {
			PositionManager.getInstance().getObject("Cube").setPose(
				new Pose2d(10.0, 5.0, new Rotation2d())
			);
			PositionManager.getInstance().getObject("Cone").setPose(
				new Pose2d(-100, -100, new Rotation2d())
			);
		}

		SmartDashboard.putNumber("/State/ArmAngle", armAngle);
		SmartDashboard.putNumber("/State/WristAngle", wristAngle);
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

	public void intake(boolean dip) {
		intake.intake(pieceMode.intakeSpeed);
		if (!dip) return;
		if (currentPosition == Positions.ground && ((Piece) configurations.get("PieceMode").getValue()) == Piece.Cone) {
			arm.setAngle(Positions.ground.getEffectors()[0]-0.3);
		} else if (currentPosition == Positions.ground && pieceMode == Piece.Cube) {
			arm.setAngle(currentPosition.getEffectors()[0]-0.1);
		} else if (currentPosition == Positions.stowed) {
			wrist.setAngle(1);
		}
	}

	public void score() {
		intake.intake(-pieceMode.intakeSpeed);
		if (currentPosition == Positions.high && pieceMode == Piece.Cone) {
			wrist.setAngle(wrist.getCurrentState().angle + 1);
		}
	}

	public void executePosition() {
		Positions current = this.getCurrentPosition();
		Positions desired = this.getDesiredPosition();

		double[] vals = desired.getEffectors();
		if (vals[1] == -100) {
			wrist.setAngle(pieceMode.pickupAngle);
		} else {
			wrist.setAngle(vals[1]);
		}
		arm.setAngle(vals[0]);

		if (current != desired && current.isHere(vals)) {
			this.setPosition(desired);
		}
	}

	public boolean armAtSetpoint() {
		return arm.atSetpoint(0.3);
	}

	public boolean wristAtSetpoint() {
		return wrist.atSetpoint(0.3);
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
		Cube(LEDMode.Cube, -Math.PI/4, -1),
		Cone(LEDMode.Cone, -Math.PI/6, -1);

		private LEDMode mode;
		private double pickupAngle;
		private double intakeSpeed;

		private Piece(LEDMode mode, double pickupAngle, double intakeSpeed) {
			this.mode = mode;
			this.pickupAngle = pickupAngle;
			this.intakeSpeed = intakeSpeed;
		}

		public double[] getIntakePosition() {
			switch(this) {
				case Cube:
					return new double[]{-0.4, -0.8};
				case Cone:
					return new double[]{-0.32, -0.8};
				default:
					return new double[]{0.0, 0.0};
			}
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
		player(3, FieldElements.hybridStations.get(1).getZ() + FieldConstants.SCORING_DISTANCE_Z * 2),
		midAuto(2, FieldElements.hybridStations.get(1).getZ() + FieldConstants.SCORING_DISTANCE_Z * 1);

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
					return new double[] {ArmConstants.LOWER_RANGE, WristConstants.UPPER_RANGE};
				case transition:
					return new double[] {-1.3, Math.PI/4};
				case ground:
					return getInstance().getPieceMode().getIntakePosition();
				case mid:
					return new double[] {.4, -100};
				case high:
					return new double[] {.51, .15};
				case player:
					return new double[] {Math.PI/4, -100};
				case midAuto:
					return new double[]{-0.5, Math.PI/3};
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