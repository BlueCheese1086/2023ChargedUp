// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.StateManager.StateManager;
import frc.robot.Wrist.WristSubsystem;

public class RobotContainer {

	public RobotContainer() {

		new DrivetrainSubsystem();
		new StateManager(new ArmSubsystem(), new WristSubsystem());
		new IntakeSubsystem();


		
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
