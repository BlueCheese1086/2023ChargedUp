// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.time.StopWatch;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Drivetrain.Commands.Drive;
import frc.robot.Drivetrain.Commands.Auto.AutoBalance;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Intake.Commands.DefaultIntake;
import frc.robot.Sensors.Feedback.VisualFeedback;
import frc.robot.Sensors.Feedback.VisualFeedback.LEDMode;
import frc.robot.Sensors.Field.PositionManager;
import frc.robot.Sensors.Vision.Vision;
import frc.robot.SparkMaxUtils.SparkMaxManager;
import frc.robot.Wrist.WristSubsystem;

public class RobotContainer {

	WristSubsystem wrist = new WristSubsystem();
	ArmSubsystem arm = new ArmSubsystem();
	IntakeSubsystem intake = new IntakeSubsystem();

	DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

	SendableChooser<Command> autoCommands = new SendableChooser<>();

	XboxController driver, operator;

	public RobotContainer() {

		new SparkMaxManager();

		PositionManager.getInstance();
		Vision.getInstance();

		driver = new XboxController(0);
		operator = new XboxController(1);

		drivetrain.setDefaultCommand(new Drive(
			drivetrain, 
			() -> filter(-driver.getLeftY()), 
			() -> filter(-driver.getLeftX()), 
			() -> filter(-driver.getRightX())
		));

		new JoystickButton(driver, Button.kA.value).whileTrue(new AutoBalance(drivetrain));

		intake.setDefaultCommand(new DefaultIntake(intake, () -> -0.3));

		StopWatch s = new StopWatch();
		s.start();

		VisualFeedback.getInstance().setMode(LEDMode.Cheese);

		new Trigger(() -> {return DriverStation.isDisabled() && DriverStation.getMatchTime() > 0;}).whileTrue(
			new InstantCommand(() -> {
				VisualFeedback.getInstance().setMode(LEDMode.Cheese);
			}, VisualFeedback.getInstance()
		));

		new Trigger(() -> {
			return DriverStation.isEnabled() && DriverStation.getMatchTime() < 30;
		}).whileTrue(
			new InstantCommand(() -> {
				VisualFeedback.getInstance().setMode(LEDMode.Bad);
			}, VisualFeedback.getInstance()
		));

		new Trigger(() -> {return DriverStation.isDisabled() && DriverStation.getMatchTime() < 1;}).whileTrue(
			new InstantCommand(() -> {
				VisualFeedback.getInstance().setMode(LEDMode.Rainbow);
			}, VisualFeedback.getInstance()
		));

		// new Trigger(() -> {
		// 	return DriverStation.isDSAttached() && DriverStation.isDisabled() && s.getDuration() < 10.0;
		// }).onTrue(new InstantCommand(() -> {
		// 	Gyro.getInstance().initGyro();
		// }).ignoringDisable(true));

		configureBindings();
	}

	private void configureBindings() {

	}

	public double filter(double d) {
		return Math.abs(d) > 0.1 ? d : 0.0;
	}

	public Command getAutonomousCommand() {
		// return new SequentialCommandGroup(
		// 	scoreMid(),
		// 	drivetrain.getPathCommand("Qual100", new PathConstraints(3, 1.5)),
		// 	new AutoBalance(drivetrain)
		// );
		return scorePickBalance();
	}

	private Command scorePickBalance() {
		return new SequentialCommandGroup(
			drivetrain.getPathCommand("Quals114", new PathConstraints(2, 1)),
			new AutoBalance(drivetrain)
		);
	}
}
