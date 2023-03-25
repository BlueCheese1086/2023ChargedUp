package frc.robot.StateManager;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Arm.ArmState;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Elevator.ElevatorState;
import frc.robot.Elevator.ElevatorSubsystem;
import frc.robot.Sensors.Feedback.VisualFeedback.LEDMode;
import frc.robot.Sensors.Field.FieldElements;
import frc.robot.Sensors.Field.PositionManager;
import frc.robot.Wrist.WristState;
import frc.robot.Wrist.WristSubsystem;

public class StateManager extends SubsystemBase {

	private static StateManager instance;

	private ElevatorSubsystem elevator;
	private ArmSubsystem arm;
	private WristSubsystem wrist;

	private final double MAX_HEIGHT = ElevatorConstants.MAX_HEIGHT;

	private final double ELEVATOR_ANGLE = ElevatorConstants.TOWER_ANGLE_OFFSET;
	private final double ARM_LENGTH = ArmConstants.ARM_LENGTH;
	private final double INTAKE_LENGTH = IntakeConstants.INTAKE_LENGTH;

	private double elevatorHeight = 0;
	private double armAngle = 0;
	private double wristAngle = 0;

	private Positions desiredPosition = Positions.stowed;
	private Positions currentPosition = Positions.stowed;

	private final HashMap<String, Object> configurations = new HashMap<>();

	private Mechanism2d totalMech = new Mechanism2d(2, 2);
	MechanismRoot2d root = totalMech.getRoot("elevator", 0, 0);
	MechanismLigament2d evLigament = root.append(new MechanismLigament2d("ev", elevatorHeight,
			Units.radiansToDegrees(Math.PI / 2 - ELEVATOR_ANGLE), 2, new Color8Bit(255, 0, 0)));
	MechanismLigament2d armLigament = evLigament.append(
			new MechanismLigament2d("arm", ARM_LENGTH, 90.0));
	MechanismLigament2d wristLigament = armLigament.append(
		new MechanismLigament2d("wrist", INTAKE_LENGTH, 90.0)
	);

	public static StateManager getInstance() {
		return instance;
	}

	public StateManager(ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem w) {
		instance = this;
		this.elevator = elevator;
		this.arm = arm;
		this.wrist = w;
		Shuffleboard.getTab("Field").add("Mech", totalMech);

		Shuffleboard.getTab("State").addDouble("Elevator", () -> elevator.getCurrentState().height);
		Shuffleboard.getTab("State").addDouble("Arm", () -> arm.getCurrentState().angle);
		Shuffleboard.getTab("State").addDouble("Wrist", () -> wrist.getCurrentState().angle);

		configurations.putAll(Map.ofEntries(
				Map.entry("piecemode", Piece.Cube),
				Map.entry("inversekinematicson", true)));
	}

	@Override
	public void periodic() {
		elevatorHeight = elevator.getCurrentState().height;
		armAngle = arm.getCurrentState().angle;
		wristAngle = wrist.getCurrentState().angle;

		evLigament.setLength(elevatorHeight);
		armLigament.setAngle(new Rotation2d(armAngle - Units.degreesToRadians(evLigament.getAngle())));
		wristLigament.setAngle(new Rotation2d(wristAngle - armAngle));
	}

	public ArmState getArmState() {
		return arm.getCurrentState();
	}

	public WristState getWristState() {
		return wrist.getCurrentState();
	}

	public ElevatorState getElevatorState() {
		return elevator.getCurrentState();
	}

	public void setPosition(Positions p) {
		desiredPosition = p;
		if (currentPosition == Positions.stowed && p != Positions.stowed) {
			currentPosition = Positions.transition;
			return;
		} else if (currentPosition != Positions.stowed && p == Positions.stowed) {
			currentPosition = Positions.transition;
			return;
		}
		currentPosition = p;
	}

	public void executePosition() {
		Positions current = this.getCurrentPosition();
		Positions desired = this.getDesiredPosition();

		while (true) {
			if ((Boolean) configurations.get("inversekinematicson")) {
				if (desired.getEffectors()[2] != -100) break;
				Pose3d nearestScoring = PositionManager.getInstance().poseOfClosestScoring();
				Pose3d robotPose = new Pose3d(PositionManager.getInstance().getRobotPose());
				double desiredWrist = ((Piece) configurations.get("piecemode")).getWristAngle();

				Pose3d endEff = new Pose3d(
					nearestScoring.getX() - IntakeConstants.INTAKE_LENGTH*Math.cos(desiredWrist),
					robotPose.getY(),
					nearestScoring.getZ() - IntakeConstants.INTAKE_LENGTH*Math.sin(desiredWrist),
					new Rotation3d()
				);

				Transform3d difference = endEff.minus(robotPose);
				SmartDashboard.putString("Difference", difference.toString());
				double[] ik = inverseIt(difference.getX(), difference.getZ());
				elevator.setDesiredHeight(ik[0]);
				arm.setAngle(ik[1]);
				wrist.setAngle(desiredWrist, true);
				return;
			}
		}

		double[] vals = desired.getEffectors();
		elevator.setDesiredHeight(vals[0]);
		arm.setAngle(vals[1]);
		if (vals[2] != -100) {
			wrist.setAngle(vals[2], true);
		} else {
			wrist.setAngle(((Piece) configurations.get("piecemode")).getWristAngle(), true);
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

	public void setConfiguration(String key, Object value) {
		configurations.replace(key, value);
	}

	public Piece getPieceMode() {
		return (Piece) configurations.get("piecemode");
	}

	public void setPieceMode(Piece p) {
		configurations.replace("piecemode", p);
	}

	public static double[] getEndPosition(double h, double a) {
		return new double[] {
				h * Math.sin(ElevatorConstants.TOWER_ANGLE_OFFSET) + ArmConstants.ARM_LENGTH * Math.cos(a),
				h * Math.cos(ElevatorConstants.TOWER_ANGLE_OFFSET) + ArmConstants.ARM_LENGTH * Math.sin(a)
		};
	}

	public double[] inverseIt(double x, double y) {
		double a1 = -Math.acos((x * Math.cos(ELEVATOR_ANGLE) - y * Math.sin(ELEVATOR_ANGLE)) / ARM_LENGTH)
				- ELEVATOR_ANGLE;
		double a2 = Math.acos((x * Math.cos(ELEVATOR_ANGLE) - y * Math.sin(ELEVATOR_ANGLE)) / ARM_LENGTH)
				- ELEVATOR_ANGLE;
		double u1 = (x - ARM_LENGTH * Math.cos(a1)) / Math.sin(ELEVATOR_ANGLE);
		double u2 = (x - ARM_LENGTH * Math.cos(a2)) / Math.sin(ELEVATOR_ANGLE);
		if (Double.isNaN(a1) || Double.isNaN(a2))
			return new double[] { elevatorHeight, armAngle };
		if (u1 > MAX_HEIGHT && u2 < 0.0)
			return new double[] { elevatorHeight, armAngle };
		return new double[] {
				u1 > MAX_HEIGHT ? u2 : u1,
				u1 > MAX_HEIGHT ? a2 : a1,
		};
	}

	public boolean isValid(double x, double y) {
		double[] ua = inverseIt(x, y);
		if (1 / Math.tan(ua[1]) * (0 - 1 / Math.sin(ua[1]) * Math.cos(ua[1] + ELEVATOR_ANGLE)) <= 2)
			return false;
		if (ua[1] > ArmConstants.RANGE - ELEVATOR_ANGLE || ua[1] < ArmConstants.RANGE + ELEVATOR_ANGLE)
			return false;
		return true;
	}

	public enum Piece {
		Cube(LEDMode.Cube, 0.0, -1),
		Cone(LEDMode.Cone, -Math.PI / 2, 1);

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
					return new double[] { 0.8, -1.5, 1 };
				case transition:
					return new double[] { 0.75, -1.3, 1 };
				case ground:
					return new double[] { 0.51, -1.15, -100 };
				case mid:
					return new double[] { 1.02, -1.05, -100 };
				case high:
					return new double[] { 0.69, 0.23, -100 };
				case player:
					return new double[] { 0.69, 0.23, -100 };
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