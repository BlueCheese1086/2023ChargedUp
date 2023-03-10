package frc.robot.StateManager;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Elevator.ElevatorSubsystem;
import frc.robot.Wrist.WristSubsystem;

/**
 * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⣦⣀⠀⢠⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀*⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⣿⣿⣿⣿⣿⣿⣦⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀*⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣦⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀*⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀*⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣿⣦⣤⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣀⣤⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀*⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣤⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀*⠀⠀⠀⠀⠀⠀⠀⠀⠀ ⠀⠀ ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⢿⣿⣦⡄⠀⠀⠀⢀⣾⠟⠃
 * ⠀*⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⡙⢿⡇⠀⠀⢀⣿⡇⠀⠀
 * ⠀*⠀ ⠀ ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⢿⣦⣤⠀⠀⣸⡿⠀⠀⠀
 * ⠀*⠀⠀⠀⠀⠀⠀⠀⠀ ⣠⣾⣧⡀⠀⠀⠀⣀⠀⢀⣠⣤⣴⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡻⢦⡙⠈⠀⠀⠻⠃⠀⠀⠀
 * ⠀*⠀⠀⠀⠀⠀ ⣠⣶⣿⣿⣿⣿⣿⣷⣶⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣶⠈⠷⠀⠀⠀⠀⠀⠀⠀
 * ⠀* ⠀⠀⢀⣀⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣤⣷⣶⠀⠀⠀⠀⠀
 * ⠠⠤⠴⠿⠿⠿⠿⣿⣿⣿⣿⣿⠿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⠿⠿⠿⠿⠿⠿⠿⠿⠟⠛⠛⠃⠀⠀⠀⠀
 *
 * 
 */

public class StateManager extends SubsystemBase {

	private static StateManager instance;

	private ElevatorSubsystem elevator;
	private ArmSubsystem arm;
	private WristSubsystem wrist;

	private final double MAX_HEIGHT = ElevatorConstants.MAX_HEIGHT;
	// private final double MAX_ANGLE = ArmConstants.UPPER_LIMIT;

	private final double ELEVATOR_ANGLE = ElevatorConstants.TOWER_ANGLE_OFFSET;
	private final double ARM_LENGTH = ArmConstants.ARM_LENGTH;

	private double currentHeight = 0;
	private double currentAngle = 0;

	private Positions desiredPosition = Positions.stowed;
	private Positions currentPosition = Positions.stowed;

	private Mechanism2d totalMech = new Mechanism2d(2, 2);
	MechanismRoot2d root = totalMech.getRoot("elevator", 0, 0);
	MechanismLigament2d evLigament = root.append(new MechanismLigament2d("ev", currentHeight, Units.radiansToDegrees(Math.PI/2-ELEVATOR_ANGLE), 2, new Color8Bit(255, 0, 0)));
	MechanismLigament2d armLigament = evLigament.append(
		new MechanismLigament2d("arm", ARM_LENGTH, 90.0)
	);

	public static StateManager getInstance() {
		return instance;
	}

	public StateManager(ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem w) {
		instance = this;
		this.elevator = elevator;
		this.arm = arm;
		this.wrist = w;
        Shuffleboard.getTab("Field").add("Mech1", totalMech);

		Shuffleboard.getTab("State").addDouble("Elevator", () -> elevator.getCurrentState().height);
		Shuffleboard.getTab("State").addDouble("Arm", () -> arm.getCurrentState().angle);
		Shuffleboard.getTab("State").addDouble("Wrist", () -> wrist.getCurrentState().angle);
	}

	@Override
	public void periodic() {
		currentHeight = elevator.getCurrentState().height;
		currentAngle = arm.getCurrentState().angle;
		
		evLigament.setLength(currentHeight);
		armLigament.setAngle(new Rotation2d(currentAngle-Units.degreesToRadians(evLigament.getAngle())));
	
		
	}

	public void setPosition(Positions p) {
		desiredPosition = p;
		if (currentPosition == Positions.stowed && p != Positions.stowed) {
			currentPosition = Positions.transition.goingIn(false);
			return;
		} else if (currentPosition != Positions.stowed && p == Positions.stowed) {
			currentPosition = Positions.transition.goingIn(true);
			return;
		}
		currentPosition = p;
	}
	
	public Positions getCurrentPosition() {
		return currentPosition;
	}

	public Positions getDesiredPosition() {
		return desiredPosition;
	}

	public static enum Positions {
        stowed(0),
        transition(1),
        ground(2),
        mid(3),
        high(4),
        player(5);

		boolean goingIn = false;

		double tolerance = 0.1;

		int index;

		Positions(int a) {
			index = a;
		}

        public double[] getValue() {
            switch (this) {
                case stowed:
                    return new double[]{0.8, -1.5, 1};
                case transition:
                    return new double[]{0.75, -1.3, 1};
                case ground:
                    return new double[]{0.51, -1.15, -100};
                case mid:
                    return new double[]{1.02, -1.05, -100};
                case high:
                    return new double[]{0.69, 0.23, -100};
                case player:
                    return new double[]{0.69, 0.23, -100};
            }
            return new double[]{};
        }

		public Positions goingIn(boolean t) {
			goingIn = t;
			return this;
		}

		public boolean getDirection() {
			return goingIn;
		}

		public boolean isHere(double[] values) {
			if (this == stowed || this == transition) return (
				(Math.abs(values[0] - getValue()[0]) < tolerance) &&
				(Math.abs(values[1] - getValue()[1]) < tolerance) &&
				(Math.abs(values[2] - getValue()[2]) < tolerance)
			);
			return (
				(Math.abs(values[0] - getValue()[0]) < tolerance) &&
				(Math.abs(values[1] - getValue()[1]) < tolerance)
			);
		}

		public boolean greaterThanX(Positions p) {
			double[] thisValue = this.getValue();
			double[] cValue = p.getValue();
			return getEndPosition(thisValue[0], thisValue[1])[0] > getEndPosition(cValue[0], cValue[1])[0];
		}
    }

	public static double[] getEndPosition(double h, double a) {
		return new double[] {
				h * Math.sin(ElevatorConstants.TOWER_ANGLE_OFFSET) + ArmConstants.ARM_LENGTH * Math.cos(a),
				h * Math.cos(ElevatorConstants.TOWER_ANGLE_OFFSET) + ArmConstants.ARM_LENGTH * Math.sin(a)
		};
	}

	public double distance(double x, double y, double x1, double y1) {
		return Math.sqrt(Math.pow(x1 - x, 2) + Math.pow(y1 - y, 2));
	}

	public double[] inverseIt(double x, double y) {
		double a1 = -Math.acos((x*Math.cos(ELEVATOR_ANGLE)-y*Math.sin(ELEVATOR_ANGLE))/ARM_LENGTH)-ELEVATOR_ANGLE;
		double a2 = Math.acos((x*Math.cos(ELEVATOR_ANGLE)-y*Math.sin(ELEVATOR_ANGLE))/ARM_LENGTH)-ELEVATOR_ANGLE;
		double u1 = (x-ARM_LENGTH*Math.cos(a1))/Math.sin(ELEVATOR_ANGLE);
		double u2 = (x-ARM_LENGTH*Math.cos(a2))/Math.sin(ELEVATOR_ANGLE);
		if (Double.isNaN(a1)||Double.isNaN(a2)) return new double[]{currentHeight, currentAngle};
		return new double[]{
			u1>MAX_HEIGHT?u2:u1,
			u1>MAX_HEIGHT?a2:a1,
		};
	}

	public boolean isValid(double x, double y) {
		double[] ua = inverseIt(x, y);
		if (1/Math.tan(ua[1])*(0-1/Math.sin(ua[1])*Math.cos(ua[1]+ELEVATOR_ANGLE)) <= 2/*TODO: MAKE BUMPER LENGTH*/) return false;
		if (ua[1] > ArmConstants.RANGE - ELEVATOR_ANGLE || ua[1] < ArmConstants.RANGE + ELEVATOR_ANGLE) return false;
		return true;
	}

}
