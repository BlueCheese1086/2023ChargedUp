package frc.robot.Wrist;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SafeSubsystem;
import frc.robot.Constants.WristConstants;
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

	public WristSubsystem() {
		wrist = new SparkMax("Wrist", config, SparkMaxRelativeConfiguration.getDefault(), SparkMaxPIDConfiguration.getDefault());
	}

	public void periodic() {
		if (!isSafe()) {
			CommandScheduler.getInstance().cancel(this.getCurrentCommand());
		}
	}

	public boolean isSafe() {
		return wrist.isConnected();
	}

	public void setAngle(double r) {
		wrist.getPIDController().setReference(r, ControlType.kPosition);
	}

	// private void setVelocity(double r) {
	// 	wrist.getPIDController().setReference(r, ControlType.kVelocity);
	// }

}