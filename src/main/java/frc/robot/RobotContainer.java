// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Drivetrain.Commands.AssistedDrive;
import frc.robot.Drivetrain.Commands.DefaultDrive;
import frc.robot.Elevator.ElevatorSubsystem;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Intake.Commands.DefaultIntake;
import frc.robot.Sensors.Commands.BeforeField;
import frc.robot.Sensors.Feedback.VisualFeedback;
import frc.robot.Sensors.Field.PositionManager;
import frc.robot.Sensors.Gyro.Gyro;
import frc.robot.Sensors.Vision.VisionManager;
import frc.robot.StateManager.StateManager;
import frc.robot.StateManager.Commands.AutoEE;
import frc.robot.StateManager.Commands.DefaultManager;
import frc.robot.StateManager.StateManager.Piece;
import frc.robot.StateManager.StateManager.Positions;
import frc.robot.Wrist.WristSubsystem;

public class RobotContainer {

	private final DrivetrainSubsystem drivetrain;
	private final ElevatorSubsystem elevator;
	private final ArmSubsystem arm;
	private final WristSubsystem wrist;
	private final IntakeSubsystem intake;
	private final StateManager stateManager;

	XboxController driver = new XboxController(0);
	XboxController operator = new XboxController(1);

	SendableChooser<Command> auto = new SendableChooser<>();

	public RobotContainer() {

		VisionManager.getInstance();
		Gyro.getInstance();
		PositionManager.getInstance();
		VisualFeedback.getInstance();

		drivetrain = new DrivetrainSubsystem();
		elevator = new ElevatorSubsystem();
		arm = new ArmSubsystem();
		wrist = new WristSubsystem();
		intake = new IntakeSubsystem();
		stateManager = new StateManager(elevator, arm, wrist);

		drivetrain.setDefaultCommand(
				new DefaultDrive(drivetrain,
						() -> filter(driver.getLeftY()),
						() -> filter(driver.getLeftX()),
						() -> filter(driver.getRightX())));

		// intake.setDefaultCommand(new DefaultIntake(intake, () -> {
		// return operator.getRightTriggerAxis() - operator.getLeftTriggerAxis();
		// }));

		// stateManager.setDefaultCommand(
		// 		new AutoEE(stateManager, () -> false, () -> 0));
		// stateManager.setDefaultCommand(
		// 	new DefaultManager()
		// );

		arm.setDefaultCommand(new InstantCommand(() -> {
			arm.setAngle((driver.getRightTriggerAxis() - driver.getLeftTriggerAxis()) * 2 * Math.PI);
			// arm.setAngle(0);
		}, arm));

		// wrist.setDefaultCommand(new InstantCommand(() -> {
		// 	wrist.setAngle((driver.getRightTriggerAxis() - driver.getLeftTriggerAxis()) * 2 * Math.PI);
		// }, wrist));

		elevator.setDefaultCommand(new InstantCommand(() -> {
			elevator.setDesiredHeight(.6 + driver.getRightTriggerAxis() - driver.getLeftTriggerAxis());
		}, elevator));


		configureBindings();
	}

	private void configureBindings() {

		/**********
		 * DRIVER *
		 **********/

		// new JoystickButton(driver, Button.kA.value).toggleOnTrue(new AssistedDrive(
		// 		drivetrain,
		// 		() -> filter(driver.getLeftY()),
		// 		() -> filter(driver.getLeftX()),
		// 		() -> filter(driver.getRightX())));

		// new JoystickButton(driver, Button.kB.value).toggleOnTrue(new InstantCommand(() -> {
		// 	stateManager.setPosition(Positions.stowed);
		// }));

		// new JoystickButton(driver, Button.kA.value).whileTrue(

		/************
		 * OPERATOR *
		 ************/

		new JoystickButton(operator, Button.kY.value).onTrue(new InstantCommand(() -> {
			stateManager.setPieceMode(stateManager.getPieceMode() == Piece.Cube ? Piece.Cone : Piece.Cube);
		}));

		new POVButton(operator, 0).onTrue(new InstantCommand(() -> {
			stateManager.setPosition(Positions.high);
		}));
		new POVButton(operator, 90).onTrue(new InstantCommand(() -> {
			stateManager.setPosition(Positions.mid);
		}));
		new POVButton(operator, 180).onTrue(new InstantCommand(() -> {
			stateManager.setPosition(Positions.ground);
		}));
		new POVButton(operator, 270).onTrue(new InstantCommand(() -> {
			stateManager.setPosition(Positions.stowed);
		}));
	}

	public double filter(double d) {
		return Math.abs(d) > 0.1 ? d : 0.0;
	}

	public Command getAutonomousCommand() {
		return new SequentialCommandGroup(new ParallelRaceGroup(new WaitCommand(0.3),
				new DefaultDrive(drivetrain,
						() -> 1,
						() -> 0.0,
						() -> 0.0)),
				new DefaultDrive(drivetrain, () -> 0.0, () -> 0.0, () -> 0.0),
				new WaitCommand(1),
				new ParallelRaceGroup(new WaitCommand(1.75),
						new DefaultDrive(drivetrain,
								() -> -0.5,
								() -> 0.0,
								() -> 0.0)));
		// return Commands.print("No autonomous command configured");
	}

	public Command getBeforeField() {
		Command c = new BeforeField();
		c.ignoringDisable(true);
		return c;
	}
}
