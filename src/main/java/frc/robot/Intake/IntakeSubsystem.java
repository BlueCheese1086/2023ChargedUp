package frc.robot.Intake;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;
import frc.robot.LiveConfiguration.ControllableConfiguration;
import frc.robot.SparkMaxUtils.SparkMax;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxConfiguration;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxPIDConfiguration;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxRelativeConfiguration;
import frc.robot.SafeSubsystem;;

public class IntakeSubsystem extends SubsystemBase implements SafeSubsystem {

	private final SparkMax intake;

    private final HashMap<String, ControllableConfiguration> configurations = new HashMap<>();

	/** Creates a new Intake. */
	public IntakeSubsystem() {
		intake = new SparkMax("Intake", new SparkMaxConfiguration(
			IntakeConstants.ID,
			MotorType.kBrushless,
			false,
			25,
			IdleMode.kBrake
		), SparkMaxRelativeConfiguration.getDefault(), SparkMaxPIDConfiguration.getDefault());

		intake.restoreFactoryDefaults();

		intake.setSmartCurrentLimit(20);
		intake.setInverted(true);

        configurations.putAll(Map.ofEntries(
            Map.entry("Enabled", new ControllableConfiguration("Subsystems", "Intake Enabled", true))
        ));
	}

    @Override
    public void periodic() {
        if (!isSafe()) {
            intake.stopMotor();
			CommandScheduler.getInstance().cancel(this.getCurrentCommand());
        }
		SmartDashboard.putNumber("/Intake/Speed", intake.get());
		SmartDashboard.putNumber("/Intake/Current", intake.getOutputCurrent());
    }

	public boolean isSafe() {
		return false;
	}

	public void stop() {
		intake.set(0);
	}

	public void intake(double speed) {
		intake.set(speed);
	}

}