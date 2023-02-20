// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Drivetrain.Commands.DefaultDrive;

public class RobotContainer {

	DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

	XboxController driver = new XboxController(0);

	public RobotContainer() {

		drivetrain.setDefaultCommand(
			new DefaultDrive(drivetrain, 
				() -> filter(driver.getLeftY()), 
				() -> filter(driver.getLeftX()), 
				() -> filter(driver.getRightX())
			));

		configureBindings();
	}

	private void configureBindings() {

		new POVButton(driver, 270).onTrue(new InstantCommand(() -> {
			drivetrain.resetGyro();
		}));


	}

	public double filter(double d) {
		return Math.abs(d) > 0.1 ? d : 0.0;
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
