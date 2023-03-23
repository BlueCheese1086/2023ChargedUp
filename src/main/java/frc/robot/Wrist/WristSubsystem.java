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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Auto.SystemsCheck.SubChecker;
import frc.robot.Auto.SystemsCheck.SystemsCheck;
import frc.robot.Constants.WristConstants;
import frc.robot.Util.DebugPID;

public class WristSubsystem extends SubsystemBase implements SubChecker {

	private WristState state;

	private CANSparkMax left, right;
	private SparkMaxPIDController controller;

	private RelativeEncoder relEncoder;
	private SparkMaxAbsoluteEncoder absoluteEncoder;

	/** Creates a new Wrist. */
	public WristSubsystem() {
		left = new CANSparkMax(Constants.WristConstants.LEFT_ID, MotorType.kBrushless);
		right = new CANSparkMax(Constants.WristConstants.RIGHT_ID, MotorType.kBrushless);

		left.restoreFactoryDefaults();
		right.restoreFactoryDefaults();

		left.setIdleMode(IdleMode.kBrake);
		right.setIdleMode(IdleMode.kBrake);

		left.follow(ExternalFollower.kFollowerDisabled, 0);
		right.follow(left, false);

		controller = left.getPIDController();

		controller.setP(Constants.WristConstants.kP);
		controller.setI(Constants.WristConstants.kI);
		controller.setD(Constants.WristConstants.kD);
		controller.setFF(Constants.WristConstants.kFF);

		relEncoder = left.getEncoder();
		absoluteEncoder = left.getAbsoluteEncoder(Type.kDutyCycle);
		absoluteEncoder.setZeroOffset(WristConstants.ENC_OFFSET);
		absoluteEncoder.setPositionConversionFactor(2 * Math.PI);// / WristConstants.GEARBOX_RATIO);

		relEncoder.setPositionConversionFactor(2 * Math.PI / Constants.WristConstants.GEARBOX_RATIO);
		relEncoder.setVelocityConversionFactor(2 * Math.PI / Constants.WristConstants.GEARBOX_RATIO / 60);
		relEncoder.setPosition(absoluteEncoder.getPosition());

		state = new WristState(relEncoder.getPosition(), relEncoder.getVelocity());

		new DebugPID(controller, "Wrist");
	}

	public void setAngle(double angle) {
		if (Robot.isSimulation()) {
            relEncoder.setPosition(angle);
            return;
        }
		controller.setReference(angle, ControlType.kPosition);
	}

	public void reset() {
		setAngle(0);
	}

	public void setVelocity(double angularSpeed) {
		controller.setReference(angularSpeed, ControlType.kVelocity);
	}

	@Override
	public void periodic() {
		state = new WristState(relEncoder.getPosition(), relEncoder.getVelocity());
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