// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.time.StopWatch;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Drivetrain.Commands.Drive;
import frc.robot.Sensors.Feedback.VisualFeedback;
import frc.robot.Sensors.Feedback.VisualFeedback.LEDMode;

public class RobotContainer {

	// WristSubsystem wrist = new WristSubsystem();
	// ArmSubsystem arm = new ArmSubsystem();
	// IntakeSubsystem intake = new IntakeSubsystem();
	// StateManager state = new StateManager(arm, wrist, intake);

	DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();

	XboxController driver, operator;

	public RobotContainer() {

		// PositionManager.getInstance();
		// Vision.getInstance();

		// drivetrain.setEvents(
		// 	Map.ofEntries(
        //     	Map.entry("AutoBalance", new AutoBalance(drivetrain)),
		// 		Map.entry("High", new InstantCommand(() -> {
		// 			arm.setAngle(0);
		// 		}, arm)),
		// 		Map.entry("IntakeDown", new AutoSetPosition(Positions.ground)),
		// 		Map.entry("IntakeCone", 
		// 			new SequentialCommandGroup(
		// 				new InstantCommand(() -> {state.setPieceMode(Piece.Cone);}),
		// 				new ParallelRaceGroup(
		// 					new WaitCommand(2),
		// 					new Intake(arm, wrist, intake, true)
		// 				),
		// 				new AutoSetPosition(Positions.stowed),
		// 				new DefaultIntake(intake, () -> -0.3).raceWith(new WaitCommand(0.1))
		// 			)

		// 		)
        // 	)
		// );
		
		

		driver = new XboxController(0);
		operator = new XboxController(1);

		drivetrain.setDefaultCommand(new Drive(
			drivetrain, 
			() -> filter(-driver.getLeftY()), 
			() -> filter(-driver.getLeftX()), 
			() -> filter(-driver.getRightX())
		));

		// new JoystickButton(driver, Button.kA.value).whileTrue(new AutoBalance(drivetrain));

		// state.setPosition(Positions.stowed);
		// state.setDefaultCommand(new DefaultManager(arm, wrist));

		// intake.setDefaultCommand(new DefaultIntake(intake, () -> -0.3));

		VisualFeedback.getInstance().setMode(LEDMode.Cheese);

		// new Trigger(() -> {return DriverStation.isDisabled() && DriverStation.getMatchTime() > 0;}).whileTrue(
		// 	new InstantCommand(() -> {
		// 		VisualFeedback.getInstance().setMode(LEDMode.Cheese);
		// 	}, VisualFeedback.getInstance()
		// ));

		// new Trigger(() -> {
		// 	return DriverStation.isEnabled() && DriverStation.getMatchTime() < 30;
		// }).whileTrue(
		// 	new InstantCommand(() -> {
		// 		VisualFeedback.getInstance().setMode(LEDMode.Bad);
		// 	}, VisualFeedback.getInstance()
		// ));

		// new Trigger(() -> {return DriverStation.isDisabled() && DriverStation.getMatchTime() < 1;}).whileTrue(
		// 	new InstantCommand(() -> {
		// 		VisualFeedback.getInstance().setMode(LEDMode.Rainbow);
		// 	}, VisualFeedback.getInstance()
		// ));

		// new Trigger(() -> {
		// 	return DriverStation.isDSAttached() && DriverStation.isDisabled() && s.getDuration() < 10.0;
		// }).onTrue(new InstantCommand(() -> {
		// 	Gyro.getInstance().initGyro();
		// }).ignoringDisable(true));

		configureBindings();
	}

	private void configureBindings() {

		// new JoystickButton(driver, Button.kRightBumper.value).whileTrue(
		// 	new InstantCommand(() -> {
		// 		state.setPosition(Positions.stowed);
		// 	}).repeatedly()
		// );

		// new JoystickButton(driver, Button.kB.value).whileTrue(
		// 	new AlignmentDrive(drivetrain, () -> filter(-driver.getLeftY())).repeatedly()
		// );

		// new POVButton(driver, 0).onTrue(
		// 	new InstantCommand(() -> {
		// 		Gyro.getInstance().setAngle(180);
		// 	})
		// );

		// new POVButton(operator, 0).onTrue(new InstantCommand(() -> {
		// 	state.setPosition(Positions.high);
		// }));
		// new POVButton(operator, 90).onTrue(new InstantCommand(() -> {
		// 	state.setPosition(state.getPieceMode() == Piece.Cone ? Positions.mid : Positions.midAuto);
		// }));
		// new POVButton(operator, 180).onTrue(new InstantCommand(() -> {
		// 	state.setPosition(Positions.ground);
		// }));
		// new POVButton(operator, 270).onTrue(new InstantCommand(() -> {
		// 	state.setPosition(Positions.stowed);
		// }));

		// new JoystickButton(operator, Button.kRightBumper.value).whileTrue(new Intake(arm, wrist, intake, true));
		// new JoystickButton(operator, Button.kLeftBumper.value).whileTrue(new Outtake(arm, wrist, intake));

		// new JoystickButton(operator, Button.kY.value).onTrue(new InstantCommand(() -> {
		// 	state.setPieceMode(state.getPieceMode() == Piece.Cone ? Piece.Cube : Piece.Cone);
		// }));
	}

	public double filter(double d) {
		return Math.abs(d) > 0.1 ? d : 0.0;
	}

	public Command getAutonomousCommand() {
		return Commands.print("Hello");
		// return new SequentialCommandGroup(
		// 	scoreMid(),
		// 	drivetrain.getPathCommand("Qual100", new PathConstraints(3, 1.5)),
		// 	new AutoBalance(drivetrain)
		// );
		// return scorePickBalance();
	}

	// private Command scorePickBalance() {
	// 	return new SequentialCommandGroup(
	// 		scoreMid(),
	// 		drivetrain.getPathCommand("Quals114", new PathConstraints(2, 1)),
	// 		new AutoBalance(drivetrain)
	// 	);
	// }

	// private Command scoreMid() {
	// 	return new SequentialCommandGroup(
	// 		// new ParallelRaceGroup(
	// 		// 	new Outtake(arm, wrist, intake),
	// 		// 	new WaitCommand(0.01)
	// 		// ),
	// 		new ParallelRaceGroup(
	// 			new Intake(arm, wrist, intake, false).repeatedly(),
	// 			new WaitCommand(1)
	// 		),
	// 		new AutoSetPosition(Positions.midAuto),
	// 		new WaitCommand(0.5),
	// 		new ParallelRaceGroup(
	// 			new Outtake(arm, wrist, intake).repeatedly(),
	// 			new WaitCommand(1)
	// 		).andThen(
	// 			new DefaultIntake(intake, () -> -0.4).raceWith(
	// 				new WaitCommand(0.1)
	// 			)
	// 		),
	// 		new AutoSetPosition(Positions.stowed)
	// 	);
	// }

	// private Command scoreCubeMid() {
	// 	return new SequentialCommandGroup(
	// 		new ParallelRaceGroup(
	// 			new Outtake(arm, wrist, intake),
	// 			new WaitCommand(0.01)
	// 		),
	// 		new ParallelRaceGroup(
	// 			new Intake(arm, wrist, intake, false).repeatedly(),
	// 			new WaitCommand(1)
	// 		),
	// 		new AutoSetPosition(Positions.mid),
	// 		new WaitCommand(0.5),
	// 		new ParallelRaceGroup(
	// 			new Outtake(arm, wrist, intake).repeatedly(),
	// 			new WaitCommand(1)
	// 		).andThen(
	// 			new DefaultIntake(intake, () -> -0.4).raceWith(
	// 				new WaitCommand(0.1)
	// 			)
	// 		),
	// 		new AutoSetPosition(Positions.stowed)
	// 	);
	// }

	// private Command quals41() {
	// 	return new SequentialCommandGroup(
	// 		new ParallelRaceGroup(
	// 			new Outtake(arm, wrist, intake),
	// 			new WaitCommand(0.01)
	// 		),
	// 		new ParallelRaceGroup(
	// 			new Intake(arm, wrist, intake, false).repeatedly(),
	// 			new WaitCommand(1)
	// 		),
	// 		new AutoSetPosition(Positions.mid),
	// 		new WaitCommand(0.5),
	// 		new ParallelRaceGroup(
	// 			new Outtake(arm, wrist, intake).repeatedly(),
	// 			new WaitCommand(1)
	// 		).andThen(
	// 			new DefaultIntake(intake, () -> -0.4).raceWith(
	// 				new WaitCommand(0.1)
	// 			)
	// 		),
	// 		new AutoSetPosition(Positions.stowed),
	// 		drivetrain.getPathCommand("OnlyBalance", new PathConstraints(3, 1.5)),
	// 		new AutoBalance(drivetrain)
	// 	);
	// }

	// private Command quals25() {
	// 	return new SequentialCommandGroup(
	// 		new ParallelRaceGroup(
	// 			new Outtake(arm, wrist, intake),
	// 			new WaitCommand(0.01)
	// 		),
	// 		new ParallelRaceGroup(
	// 			new Intake(arm, wrist, intake, false).repeatedly(),
	// 			new WaitCommand(1)
	// 		),
	// 		new AutoSetPosition(Positions.mid),
	// 		new WaitCommand(0.5),
	// 		new ParallelRaceGroup(
	// 			new Outtake(arm, wrist, intake).repeatedly(),
	// 			new WaitCommand(1)
	// 		).andThen(
	// 			new DefaultIntake(intake, () -> -0.4).raceWith(
	// 				new WaitCommand(0.1)
	// 			)
	// 		),
	// 		new AutoSetPosition(Positions.stowed),
	// 		drivetrain.getPathCommand("1PieceBalance", new PathConstraints(3, 1.5)),
	// 		new AutoBalance(drivetrain)
	// 	);
	// }

	// private Command quals18() {
	// 	return new SequentialCommandGroup(
	// 		new ParallelRaceGroup(
	// 			new Outtake(arm, wrist, intake),
	// 			new WaitCommand(0.01)
	// 		),
	// 		new ParallelRaceGroup(
	// 			new Intake(arm, wrist, intake, false).repeatedly(),
	// 			new WaitCommand(1)
	// 		),
	// 		new AutoSetPosition(Positions.mid),
	// 		new WaitCommand(0.5),
	// 		new ParallelRaceGroup(
	// 			new Outtake(arm, wrist, intake).repeatedly(),
	// 			new WaitCommand(1)
	// 		).andThen(
	// 			new DefaultIntake(intake, () -> -0.4).raceWith(
	// 				new WaitCommand(0.1)
	// 			)
	// 		),
	// 		new AutoSetPosition(Positions.stowed),
	// 		new WaitCommand(3),
	// 		drivetrain.getPathCommand("1PieceFollowBees", new PathConstraints(3, 1.5))
	// 		// new AutoBalance(drivetrain)
	// 	);
	// }
}
