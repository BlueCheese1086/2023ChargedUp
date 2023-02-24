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

	private final double MAX_HEIGHT = ElevatorConstants.MAX_HEIGHT;
	// private final double MAX_ANGLE = ArmConstants.UPPER_LIMIT;

	private final double ELEVATOR_ANGLE = ElevatorConstants.TOWER_ANGLE_OFFSET;
	private final double ARM_LENGTH = ArmConstants.ARM_LENGTH;

	private double currentHeight = 0;
	private double currentAngle = 0;

	private Mechanism2d totalMech = new Mechanism2d(2, 2);
	MechanismRoot2d root = totalMech.getRoot("elevator", 0, 0);
	MechanismLigament2d evLigament = root.append(new MechanismLigament2d("ev", currentHeight, Units.radiansToDegrees(Math.PI/2-ELEVATOR_ANGLE), 2, new Color8Bit(255, 0, 0)));
	MechanismLigament2d armLigament = evLigament.append(
		new MechanismLigament2d("arm", ARM_LENGTH, 90.0)
	);

	public static StateManager getInstance() {
		return instance;
	}

	public StateManager(ElevatorSubsystem elevator, ArmSubsystem arm) {
		instance = this;
		this.elevator = elevator;
		this.arm = arm;
        Shuffleboard.getTab("Field").add("Mech", totalMech);
	}

	@Override
	public void periodic() {
		currentHeight = elevator.getCurrentState().height;
		currentAngle = arm.getCurrentState().angle;
		
		evLigament.setLength(currentHeight);
		armLigament.setAngle(new Rotation2d(currentAngle-Units.degreesToRadians(evLigament.getAngle())));
	}

	public double[] getEndPosition(double h, double a) {
		return new double[] {
				h * Math.sin(ELEVATOR_ANGLE) + ARM_LENGTH * Math.cos(a),
				h * Math.cos(ELEVATOR_ANGLE) + ARM_LENGTH * Math.sin(a)
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

}
