package frc.robot.Intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Auto.SystemsCheck.SubChecker;
import frc.robot.Auto.SystemsCheck.SystemsCheck;

import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase implements SubChecker {
	CANSparkMax intake;
	RelativeEncoder intakeEncoder;

	private static IntakeSubsystem instance;

	public static IntakeSubsystem getInstance() {
		if (instance == null) {
			instance = new IntakeSubsystem();
		}
		return instance;
	}

	/** Creates a new Intake. */
	public IntakeSubsystem() {
		intake = new CANSparkMax(Constants.IntakeConstants.ID, MotorType.kBrushless);
		intakeEncoder = intake.getEncoder();
		instance = this;
	}

	public void in() {
		intake.set(0.5);
	}

	public void out() {
		intake.set(-0.5);
	}

	public void stop() {
		intake.set(0);
	}

	public void intake(double speed) {
		intake.set(speed);
	}

	public AbsoluteEncoder getWristEncoder() {
		return intake.getAbsoluteEncoder(Type.kDutyCycle);
	}

	public Command check(boolean safe) {
		if (!safe) {
			return new InstantCommand();
		}
		return new InstantCommand(() -> {
			double time = System.currentTimeMillis();
			boolean working = false;
			intake(0.3);
			while (System.currentTimeMillis() - time < 1000) {
			}
			if (intakeEncoder.getVelocity() > 0.1) {
				working = true;
			}
			SystemsCheck.setSystemStatus(this, working);
			stop();
		}, this);
	}

}