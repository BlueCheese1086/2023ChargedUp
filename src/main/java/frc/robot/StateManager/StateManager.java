package frc.robot.StateManager;

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

	/**
	 * NUMERICAL SOLUTION
	 * ITERATE THROUGH DIFFERENT SOLUTIONS TO FIND THE BEST
	 */

	private ElevatorSubsystem elevator;
	private ArmSubsystem arm;

	private final double MAX_HEIGHT = ElevatorConstants.MAX_HEIGHT;
	private final double MAX_ANGLE = ArmConstants.UPPER_LIMIT;

	private final double ELEVATOR_ANGLE = ElevatorConstants.TOWER_ANGLE_OFFSET;
	private final double ARM_LENGTH = ArmConstants.ARM_LENGTH;

	private final double ELEVATOR_STEP = 1 / 42 * ElevatorConstants.SPOOL_RADIUS * 2 * Math.PI
			/ ElevatorConstants.GEARBOX_RATIO;
	private final double ARM_STEP = 1 / 42 * 360 / ArmConstants.GEARBOX_RATIO;

	private double currentHeight = 0;
	private double currentAngle = 0;

	public StateManager(ElevatorSubsystem elevator, ArmSubsystem arm) {
		this.elevator = elevator;
		this.arm = arm;
	}

	@Override
	public void periodic() {
		currentHeight = elevator.getCurrentState().height;
		// currentAngle = arm.getCurrentState().angle;
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

	public double[] iterate(double x, double y) {
		double u = y;
		double a = 0;

		double[] closestVars = new double[] { u, a };
		double lastD = distance(x, y, getEndPosition(u, a)[0], getEndPosition(u, a)[1]);
		double distanceChange = 0.0;

		while (distanceChange < 0.0) {
			distanceChange = 0;
			for (int i = -1; i <= 1; i += 2) {
				for (int j = -1; j <= 1; j += 2) {
					double[] endPos = getEndPosition(u + i * ELEVATOR_STEP, a + j * ARM_STEP);
					double currentD = distance(x, y, endPos[0], endPos[1]);
					if (currentD < lastD) {
						u = u + i * ELEVATOR_STEP;
						a = a + j * ARM_STEP;
						distanceChange = currentD - lastD;
					}
				}
			}
		}
		return new double[]{u, a};
	}

}
