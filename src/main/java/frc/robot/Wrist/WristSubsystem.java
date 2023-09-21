package frc.robot.Wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;


import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SafeSubsystem;
import frc.robot.Constants.WristConstants;
import frc.robot.LiveConfiguration.DebugPID;
import frc.robot.SparkMaxUtils.SparkMax;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxConfiguration;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxPIDConfiguration;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxRelativeConfiguration;

public class WristSubsystem extends SubsystemBase implements SafeSubsystem {

	private final SparkMax wrist;
	private final SparkMaxConfiguration config = new SparkMaxConfiguration(
		WristConstants.ID,
		MotorType.kBrushless,
		true,
		45,
		IdleMode.kBrake
	);

	private final AbsoluteEncoder absoluteEncoder;
	private final RelativeEncoder relativeEncoder;
	private final SparkMaxPIDController pidController;

	private WristState state = new WristState(0.0, 0.0);

	public WristSubsystem() {
		wrist = new SparkMax("Wrist", config, SparkMaxRelativeConfiguration.getDefault(), SparkMaxPIDConfiguration.getDefault());

		absoluteEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);
		absoluteEncoder.setZeroOffset(WristConstants.ENC_OFFSET);
		absoluteEncoder.setPositionConversionFactor(WristConstants.GEARBOX_RATIO);
		absoluteEncoder.setVelocityConversionFactor(WristConstants.GEARBOX_RATIO/60);

		relativeEncoder = wrist.getEncoder();
		pidController = wrist.getPIDController();

		new DebugPID(pidController, "Wrist PID");
	}

	public void periodic() {
		if (!isSafe()) {
			wrist.stopMotor();
			CommandScheduler.getInstance().cancel(this.getCurrentCommand());
		}
		state = new WristState(absoluteEncoder.getPosition(), absoluteEncoder.getVelocity());
	}

	public boolean isSafe() {
		return wrist.isConnected();
	}

	public WristState getState() {
		return state;
	}

	public void setAngle(double r) {
		pidController.setReference(r, ControlType.kPosition);
	}

	// private void setVelocity(double r) {
	// 	wrist.getPIDController().setReference(r, ControlType.kVelocity);
	// }

}