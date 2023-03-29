// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Configuration.ControllableConfiguration;

public class RobotContainer {

	public RobotContainer() {

		configureBindings();
	}

	@SuppressWarnings("unchecked")
	private void configureBindings() {
		// new ControllableConfiguration("ABC", 
		// Map.entry("Name", 0.0)
		// );
		NetworkTableEntry e = new NetworkTableEntry(NetworkTableInstance.getDefault(), 0);
		e.setBoolean(true
        NetworkTableInstance.getDefault().getTable("Hello").putValue("Hello", e);
	}

	public double filter(double d) {
		return Math.abs(d) > 0.1 ? d : 0.0;
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
