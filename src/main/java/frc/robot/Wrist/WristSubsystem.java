package frc.robot.Wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Auto.SystemsCheck.SubChecker;
import frc.robot.Auto.SystemsCheck.SystemsCheck;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase implements SubChecker {

	WristState state;

	CANSparkMax left, right;

	SparkMaxPIDController leftPID;

	RelativeEncoder leftEncoder;

	AbsoluteEncoder absoluteEncoder;

	/** Creates a new Wrist. */
	public WristSubsystem() {
		left = new CANSparkMax(Constants.WristConstants.LEFT_ID, MotorType.kBrushless);
		right = new CANSparkMax(Constants.WristConstants.RIGHT_ID, MotorType.kBrushless);

		left.restoreFactoryDefaults();
		right.restoreFactoryDefaults();

		right.follow(left, false);

		leftPID = left.getPIDController();

		leftPID.setP(Constants.WristConstants.kP);
		leftPID.setI(Constants.WristConstants.kI);
		leftPID.setD(Constants.WristConstants.kD);
		leftPID.setFF(Constants.WristConstants.kFF);

		leftEncoder = left.getEncoder();
		absoluteEncoder = left.getAbsoluteEncoder(Type.kDutyCycle);
		absoluteEncoder.setZeroOffset(WristConstants.ENC_OFFSET);

		// double startPosition = absoluteEncoder.getPosition() * 2 * Math.PI;

		leftEncoder.setPositionConversionFactor(2 * Math.PI / Constants.WristConstants.GEARBOX_RATIO);
		leftEncoder.setVelocityConversionFactor(2 * Math.PI / Constants.WristConstants.GEARBOX_RATIO / 60);
		leftEncoder.setPosition(absoluteEncoder.getPosition()-absoluteEncoder.getZeroOffset());

		state = new WristState(leftEncoder.getPosition(), 0);
	}

	public void setPosition(double angle) {
		leftPID.setReference(angle, ControlType.kPosition);
	}

	public void reset() {
		setPosition(0);
	}

	public void setVelocity(double angularSpeed) {
		leftPID.setReference(angularSpeed, ControlType.kVelocity);
	}

	@Override
	public void periodic() {
		state = new WristState(leftEncoder.getPosition(), leftEncoder.getVelocity());
	}

	public Command check(boolean safe) {
		if (!safe) {
			return new InstantCommand();
		}
		return new InstantCommand(() -> {
			double time = System.currentTimeMillis();
			boolean working = false;
			setPosition(Math.PI / 6.0);
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