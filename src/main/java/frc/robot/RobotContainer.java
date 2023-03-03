// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Drivetrain.Commands.DefaultDrive;
import frc.robot.Elevator.ElevatorSubsystem;
import frc.robot.Sensors.Vision.VisionManager;
import frc.robot.StateManager.StateManager;
import frc.robot.StateManager.Commands.ElevatorArmControl;
import frc.robot.StateManager.Commands.LinearFollow;
import frc.robot.StateManager.StateManager.Positions;
import frc.robot.Wrist.WristSubsystem;

public class RobotContainer {

	// DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
	ElevatorSubsystem elevator = new ElevatorSubsystem(false);
	ArmSubsystem arm = new ArmSubsystem(true);
	WristSubsystem wrist = new WristSubsystem();
	StateManager stateManager = new StateManager(elevator, arm, wrist);

	XboxController driver = new XboxController(0);

	public RobotContainer() {

		VisionManager.getInstance();

		// new StateManager(elevator, arm);

		// drivetrain.setDefaultCommand(
		// 	new DefaultDrive(drivetrain,
		// 		() -> filter(driver.getRawAxis(0)), 
		// 		() -> filter(-driver.getRawAxis(1)), 
		// 		() -> filter(-new Joystick(1).getRawAxis(0))
		// ));

		stateManager.setDefaultCommand(new ElevatorArmControl(stateManager, arm, elevator, wrist, null));

		elevator.setDefaultCommand(new LinearFollow(elevator, arm, () -> -new Joystick(1).getRawAxis(1)));

		configureBindings();
	}

	private void configureBindings() {
		// new JoystickButton(driver, 1).onTrue(drivetrain.getFollowCommand("PANAMA"));

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
		return Commands.print("No autonomous command configured");
	}
}
