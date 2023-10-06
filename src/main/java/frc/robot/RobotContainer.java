// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Wrist.WristSubsystem;

public class RobotContainer {

	WristSubsystem wrist;// = new WristSubsystem();
	ArmSubsystem arm;// = new ArmSubsystem();
	IntakeSubsystem intake;// = new IntakeSubsystem();

	DrivetrainSubsystem drivetrain;// = new DrivetrainSubsystem();

	SendableChooser<Command> autoCommands = new SendableChooser<>();

	XboxController driver, operator;

	public RobotContainer() {

		wrist = new WristSubsystem();
		arm = new ArmSubsystem();
		intake = new IntakeSubsystem();
		drivetrain = new DrivetrainSubsystem();

		configureBindings();
	}

	private void configureBindings() {

	}

	// public double filter(double d) {}
}
