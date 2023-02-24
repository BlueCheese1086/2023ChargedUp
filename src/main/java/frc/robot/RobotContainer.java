// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Drivetrain.Commands.DefaultDrive;
import frc.robot.Elevator.ElevatorSubsystem;
import frc.robot.Elevator.Commands.RawControl;
import frc.robot.Sensors.Vision.VisionManager;
import frc.robot.StateManager.StateManager;
import frc.robot.StateManager.Commands.LinearFollow;

public class RobotContainer {

	DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
	ElevatorSubsystem elevator = new ElevatorSubsystem(false);
	ArmSubsystem arm = new ArmSubsystem(true);

	XboxController driver = new XboxController(0);

	public RobotContainer() {

		VisionManager.getInstance();
		
		new StateManager(elevator, arm);

		drivetrain.setDefaultCommand(
			new DefaultDrive(drivetrain, 
				() -> filter(driver.getLeftY()), 
				() -> filter(driver.getLeftX()), 
				() -> filter(-new Joystick(1).getRawAxis(0))
			));

		elevator.setDefaultCommand(new LinearFollow(elevator, arm, () -> -new Joystick(1).getRawAxis(1)));

		configureBindings();
	}

	private void configureBindings() {
	}

	public double filter(double d) {
		return Math.abs(d) > 0.1 ? d : 0.0;
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
