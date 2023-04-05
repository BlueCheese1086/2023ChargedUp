package frc.robot.Wrist;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Configuration.ControllableConfiguration;
import frc.robot.Configuration.DebugPID;
import frc.robot.Constants.WristConstants;
import frc.robot.SparkMaxUtils.SparkMax;
import frc.robot.StateManager.StateManager;

public class WristSubsystem extends SubsystemBase {

	private WristState state;

	private SparkMax master, follow;
	private SparkMaxPIDController controller;

	private RelativeEncoder relEncoder;
	private SparkMaxAbsoluteEncoder absoluteEncoder;

    private final HashMap<String, ControllableConfiguration> configurations = new HashMap<>();

	/** Creates a new Wrist. */
	public WristSubsystem() {
		follow = new SparkMax("Intake Follow", WristConstants.LEFT_ID, MotorType.kBrushless);
		master = new SparkMax("Intake Master", WristConstants.RIGHT_ID, MotorType.kBrushless);

		follow.restoreFactoryDefaults();
		master.restoreFactoryDefaults();

		follow.setIdleMode(IdleMode.kBrake);
		master.setIdleMode(IdleMode.kBrake);

		master.setSmartCurrentLimit(20);

		master.setInverted(true);
		follow.follow(master, false);

		relEncoder = master.getEncoder();
		absoluteEncoder = master.getAbsoluteEncoder(Type.kDutyCycle);
		absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
		absoluteEncoder.setInverted(true);
		absoluteEncoder.setZeroOffset(WristConstants.ENC_OFFSET);

		controller = master.getPIDController();

		controller.setFeedbackDevice(absoluteEncoder);

		controller.setP(WristConstants.kP);
		controller.setI(WristConstants.kI);
		controller.setD(WristConstants.kD);
		controller.setFF(WristConstants.kFF);

		state = new WristState(absoluteEncoder.getPosition(), absoluteEncoder.getVelocity());

		Shuffleboard.getTab("Wrist").addNumber("ABS", () -> absoluteEncoder.getPosition());

        Shuffleboard.getTab("Subsystems").addBoolean("Wrist", () -> true);
		new DebugPID(controller, "Wrist");

        configurations.putAll(Map.ofEntries(
            Map.entry("Enabled", new ControllableConfiguration("Subsystems", "Wrist Enabled", true))
        ));
	}

    @Override
	public void periodic() {
		if (!(Boolean) configurations.get("Enabled").getValue() || (master.getFault(FaultID.kStall) && master.getOutputCurrent() > 1)) {
            master.stopMotor();
        }
		state = new WristState(absoluteEncoder.getPosition() - Math.PI - StateManager.getInstance().getArmState().angle, absoluteEncoder.getVelocity());
	}

	public void setAngle(double angle, boolean fourBar) {
		if (Robot.isSimulation()) {
            relEncoder.setPosition(angle);
            return;
        }
		double targetAngle = angle + Math.PI + (fourBar ? -StateManager.getInstance().getArmState().angle : 0.0);
		if (targetAngle > WristConstants.UPPER_RANGE + Math.PI) {
			targetAngle = WristConstants.UPPER_RANGE + Math.PI;
		} else if (targetAngle < WristConstants.LOWER_RANGE + Math.PI) {
			targetAngle = WristConstants.LOWER_RANGE + Math.PI;
		}
		controller.setReference(targetAngle, ControlType.kPosition);
	}

	public void reset() {
		setAngle(0, false);
	}

	public void setVelocity(double angularSpeed) {
		controller.setReference(angularSpeed, ControlType.kVelocity);
	}

	public WristState getCurrentState() {
		return state;
	}
}