package frc.robot.Wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Auto.SystemsCheck.SubChecker;
import frc.robot.Auto.SystemsCheck.SystemsCheck;
import frc.robot.Constants.WristConstants;
import frc.robot.StateManager.StateManager;
import frc.robot.Util.DebugPID;

public class WristSubsystem extends SubsystemBase implements SubChecker {

	private WristState state;

	private CANSparkMax master, follow;
	private SparkMaxPIDController controller;

	private RelativeEncoder relEncoder;
	private SparkMaxAbsoluteEncoder absoluteEncoder;

	/** Creates a new Wrist. */
	public WristSubsystem() {
		follow = new CANSparkMax(Constants.WristConstants.LEFT_ID, MotorType.kBrushless);
		master = new CANSparkMax(Constants.WristConstants.RIGHT_ID, MotorType.kBrushless);

		follow.restoreFactoryDefaults();
		master.restoreFactoryDefaults();

		follow.setIdleMode(IdleMode.kBrake);
		master.setIdleMode(IdleMode.kBrake);

		master.setInverted(true);
		follow.follow(master, false);

		relEncoder = master.getEncoder();
		absoluteEncoder = master.getAbsoluteEncoder(Type.kDutyCycle);
		absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
		absoluteEncoder.setInverted(true);
		absoluteEncoder.setZeroOffset(WristConstants.ENC_OFFSET);

		controller = master.getPIDController();

		controller.setFeedbackDevice(absoluteEncoder);

		controller.setP(Constants.WristConstants.kP);
		controller.setI(Constants.WristConstants.kI);
		controller.setD(Constants.WristConstants.kD);
		controller.setFF(Constants.WristConstants.kFF);

		state = new WristState(absoluteEncoder.getPosition(), absoluteEncoder.getVelocity());

		Shuffleboard.getTab("Wrist").addNumber("ABS", () -> absoluteEncoder.getPosition());

		new DebugPID(controller, "Wrist");
	}

	public void setAngle(double angle) {
		if (Robot.isSimulation()) {
            relEncoder.setPosition(angle);
            return;
        }
		controller.setReference(angle + Math.PI, ControlType.kPosition);
	}

	public void reset() {
		setAngle(0);
	}

	public void setVelocity(double angularSpeed) {
		controller.setReference(angularSpeed, ControlType.kVelocity);
	}

	@Override
	public void periodic() {
		state = new WristState(absoluteEncoder.getPosition(), absoluteEncoder.getVelocity());
	}

	public WristState getCurrentState() {
		return state;
	}

	public Command check(boolean safe) {
		if (!safe) {
			return new InstantCommand();
		}
		return new InstantCommand(() -> {
			double time = System.currentTimeMillis();
			boolean working = false;
			setAngle(Math.PI / 6.0);
			while (System.currentTimeMillis() - time < 2000) {
			}
			if (relEncoder.getPosition() > Math.PI / 12.0) {
				working = true;
			}
			SystemsCheck.setSystemStatus(this, working);
			reset();
		}, this);
	}

}