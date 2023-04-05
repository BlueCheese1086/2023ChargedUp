// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix.time.StopWatch;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Drivetrain.Commands.Drive;
import frc.robot.Drivetrain.Commands.Auto.AutoBalance;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Intake.Commands.DefaultIntake;
import frc.robot.Sensors.Field.PositionManager;
import frc.robot.Sensors.Gyro.Gyro;
import frc.robot.StateManager.StateManager;
import frc.robot.StateManager.StateManager.Piece;
import frc.robot.StateManager.StateManager.Positions;
import frc.robot.Wrist.WristSubsystem;

public class RobotContainer {

	WristSubsystem wrist = new WristSubsystem();
	ArmSubsystem arm = new ArmSubsystem();
	IntakeSubsystem intake = new IntakeSubsystem();
	StateManager state = new StateManager(arm, wrist);

	DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

	SendableChooser<Command> autoCommands = new SendableChooser<>();

	XboxController driver;

	public RobotContainer() {

		PositionManager.getInstance();

		drivetrain.setEvents(
			Map.ofEntries(
            	Map.entry("AutoBalance", new AutoBalance(drivetrain)),
				Map.entry("High", new InstantCommand(() -> {
					arm.setAngle(0);
				}, arm))
        	)
		);

		driver = new XboxController(0);

		drivetrain.setDefaultCommand(new Drive(
			drivetrain, 
			() -> filter(-driver.getLeftY()), 
			() -> filter(-driver.getLeftX()), 
			() -> filter(-driver.getRightX())
		));

		// state.setPosition(Positions.high);
		// state.setDefaultCommand(new DefaultManager(arm, wrist));

		// intake.setDefaultCommand(new DefaultIntake(intake, () -> -0.3));
		
		// ControllableConfiguration kp = new ControllableConfiguration("Pathing", "P", 20.0);
		// ControllableConfiguration ki = new ControllableConfiguration("Pathing", "I", 0.0);
		// ControllableConfiguration kd = new ControllableConfiguration("Pathing", "D", 5.0);

		new JoystickButton(driver, Button.kA.value).onTrue(
			new InstantCommand(() -> {
				drivetrain.getPathCommand(
					"1 Piece", 
					new PathConstraints(1, 1)
				).schedule();
			})
		);

		StopWatch s = new StopWatch();
		s.start();

		new Trigger(() -> {
			return DriverStation.isDSAttached() && DriverStation.isDisabled() && s.getDuration() < 10.0;
		}).onTrue(new InstantCommand(() -> {
			Gyro.getInstance().initGyro();
		}).ignoringDisable(true));

		configureBindings();
	}

	private void configureBindings() {
		new POVButton(driver, 0).onTrue(new InstantCommand(() -> {
			state.setPosition(Positions.high);
		}));
		new POVButton(driver, 90).onTrue(new InstantCommand(() -> {
			state.setPosition(Positions.mid);
		}));
		new POVButton(driver, 180).onTrue(new InstantCommand(() -> {
			state.setPosition(Positions.ground);
		}));
		new POVButton(driver, 270).onTrue(new InstantCommand(() -> {
			state.setPosition(Positions.stowed);
		}));

		new JoystickButton(driver, Button.kRightBumper.value).whileTrue(new DefaultIntake(intake, () -> -1));
		new JoystickButton(driver, Button.kLeftBumper.value).whileTrue(new DefaultIntake(intake, () -> 1));

		new JoystickButton(driver, Button.kY.value).onTrue(new InstantCommand(() -> {
			state.setPieceMode(state.getPieceMode() == Piece.Cone ? Piece.Cube : Piece.Cone);
		}));
	}

	public double filter(double d) {
		return Math.abs(d) > 0.1 ? d : 0.0;
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
