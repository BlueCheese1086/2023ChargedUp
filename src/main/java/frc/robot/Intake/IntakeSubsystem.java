package frc.robot.Intake;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;
import frc.robot.SparkMaxUtils.SparkMax;
import frc.robot.Configuration.ControllableConfiguration;;

public class IntakeSubsystem extends SubsystemBase {
	SparkMax intake;
	RelativeEncoder intakeEncoder;

    private final HashMap<String, ControllableConfiguration> configurations = new HashMap<>();

	/** Creates a new Intake. */
	public IntakeSubsystem() {
		intake = new SparkMax("Intake Motor", IntakeConstants.ID, MotorType.kBrushless);
		intakeEncoder = intake.getEncoder();

		intake.restoreFactoryDefaults();

		intake.setSmartCurrentLimit(20);
		intake.setInverted(true);

        configurations.putAll(Map.ofEntries(
            Map.entry("Enabled", new ControllableConfiguration("Subsystems", "Intake Enabled", true))
        ));
	}

    @Override
    public void periodic() {
        if (!(Boolean) configurations.get("Enabled").getValue()) {
            intake.stopMotor();
        }
		SmartDashboard.putNumber("/Intake/Speed", intake.get());
		SmartDashboard.putNumber("/Intake/Current", intake.getOutputCurrent());
    }

	public void stop() {
		intake.set(0);
	}

	public void intake(double speed) {
		intake.set(speed);
	}

}