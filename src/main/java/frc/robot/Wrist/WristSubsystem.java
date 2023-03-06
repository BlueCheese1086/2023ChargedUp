package frc.robot.Wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Auto.SystemsCheck.SubChecker;
import frc.robot.Auto.SystemsCheck.SystemsCheck;
import frc.robot.Constants.WristConstants;
import frc.robot.Intake.IntakeSubsystem;

public class WristSubsystem extends SubsystemBase implements SubChecker {

	WristState state;

	CANSparkMax left, right;

	SparkMaxPIDController leftPID;

	RelativeEncoder leftEncoder;

	// AbsoluteEncoder absoluteEncoder;

	/** Creates a new Wrist. */
	public WristSubsystem() {
		left = new CANSparkMax(Constants.WristConstants.LEFT_ID, MotorType.kBrushless);
		right = new CANSparkMax(Constants.WristConstants.RIGHT_ID, MotorType.kBrushless);

		left.restoreFactoryDefaults();
		right.restoreFactoryDefaults();

		left.setIdleMode(IdleMode.kBrake);
		right.setIdleMode(IdleMode.kBrake);

		left.setInverted(false);
		right.setInverted(false);
		right.follow(left, false);

		leftPID = left.getPIDController();

		leftEncoder = left.getEncoder();
		// absoluteEncoder = IntakeSubsystem.getInstance().getWristEncoder();
		// absoluteEncoder.setZeroOffset(WristConstants.ENC_OFFSET);

		// leftPID.setFeedbackDevice(absoluteEncoder);
		leftPID.setP(Constants.WristConstants.kP);
		leftPID.setI(Constants.WristConstants.kI);
		leftPID.setD(Constants.WristConstants.kD);
		leftPID.setFF(Constants.WristConstants.kFF);
		// double startPosition = absoluteEncoder.getPosition() * 2 * Math.PI;

		leftEncoder.setPositionConversionFactor(2 * Math.PI / Constants.WristConstants.GEARBOX_RATIO);
		leftEncoder.setVelocityConversionFactor(2 * Math.PI / Constants.WristConstants.GEARBOX_RATIO / 60);
		leftEncoder.setPosition(Units.degreesToRadians(72));
		// leftEncoder.setPosition(absoluteEncoder.getPosition()-absoluteEncoder.getZeroOffset());

		state = new WristState(0, 0);
	}

	public void setAngle(double angle) {
		leftPID.setReference(angle, ControlType.kPosition);
	}

	public void reset() {
		setAngle(0);
	}

	public void setVelocity(double angularSpeed) {
		leftPID.setReference(angularSpeed, ControlType.kVelocity);
	}

	@Override
	public void periodic() {
		state = new WristState(leftEncoder.getPosition(), leftEncoder.getVelocity());
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
			if (leftEncoder.getPosition() > Math.PI / 12.0) {
				working = true;
			}
			SystemsCheck.setSystemStatus(this, working);
			reset();
		}, this);
	}

}