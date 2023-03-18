// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Auto.AutoLevel;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Drivetrain.Commands.DefaultDrive;
import frc.robot.Elevator.ElevatorSubsystem;
import frc.robot.Elevator.Commands.RawControl;
import frc.robot.Sensors.Commands.BeforeField;
import frc.robot.Sensors.Feedback.VisualFeedback;
import frc.robot.Sensors.Field.PositionManager;
import frc.robot.Sensors.Gyro.Gyro;
import frc.robot.Sensors.Vision.VisionManager;
import frc.robot.StateManager.StateManager;
import frc.robot.StateManager.Commands.AutoEE;
import frc.robot.StateManager.Commands.ElevatorArmControl;
import frc.robot.StateManager.StateManager.Positions;
import frc.robot.Wrist.WristSubsystem;

public class RobotContainer {

	private final DrivetrainSubsystem drivetrain;
	private final ElevatorSubsystem elevator;
	private final ArmSubsystem arm;
	private final WristSubsystem wrist;
	private final StateManager stateManager;
	private final VisualFeedback leds;

	XboxController driver = new XboxController(0);

	SendableChooser<Command> auto = new SendableChooser<>();

	public RobotContainer() {

		VisionManager.getInstance();
		Gyro.getInstance();
		PositionManager.getInstance();

		drivetrain = new DrivetrainSubsystem();
		elevator = new ElevatorSubsystem();
		arm = new ArmSubsystem();
		wrist = new WristSubsystem();
		stateManager = new StateManager(elevator, arm, wrist);
		leds = new VisualFeedback();

		drivetrain.setDefaultCommand(
			new DefaultDrive(drivetrain,
				() -> filter(driver.getLeftY()), 
				() -> filter(driver.getLeftX()), 
				() -> filter(driver.getRightX())
		));	

		// stateManager.setDefaultCommand(new ElevatorArmControl(stateManager, arm, elevator, wrist, null));

		stateManager.setDefaultCommand(new AutoEE(stateManager, () -> driver.getBButton(), () -> 0));

		// elevator.setDefaultCommand(new RawControl(elevator, () -> driver.getRightTriggerAxis() - driver.getLeftTriggerAxis()));

		configureBindings();
	}

	private void configureBindings() {
		// new JoystickButton(driver, 1).onTrue(new InstantCommand(() -> {
		// 	drivetrain.getFollowCommand("2 Piece + Pick").schedule();
		// }));

		new JoystickButton(driver, Button.kA.value).onTrue(new InstantCommand(() -> {
			stateManager.setPosition(Positions.stowed);
		}));
		new JoystickButton(driver, Button.kRightBumper.value).onTrue(new InstantCommand(() -> {
			stateManager.setPosition(Positions.ground);
		}));
		new JoystickButton(driver, Button.kB.value).onTrue(new InstantCommand(() -> {
			stateManager.setPosition(Positions.mid);
		}));
		new JoystickButton(driver, Button.kY.value).onTrue(new InstantCommand(() -> {
			stateManager.setPosition(Positions.high);
		}));
		new JoystickButton(driver, Button.kX.value).onTrue(new InstantCommand(() -> {
			stateManager.setPosition(Positions.player);
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
				() -> 0.0))
		);
		// return Commands.print("No autonomous command configured");
	}

	public Command getBeforeField() {
		Command c = new BeforeField();
		c.ignoringDisable(true);
		return c;
	}
}
