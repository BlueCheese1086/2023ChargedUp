package frc.robot.StateManager.Commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Arm.Commands.SetArmAngle;
import frc.robot.Elevator.ElevatorSubsystem;
import frc.robot.Wrist.WristSubsystem;
import frc.robot.Wrist.Commands.SetWristAngle;

public class UnfoldStartingPos extends SequentialCommandGroup {

    public UnfoldStartingPos(ArmSubsystem a, WristSubsystem w, ElevatorSubsystem e) {
        addCommands(
            new SetWristAngle(() -> Units.degreesToRadians(20), w, false),
            new ParallelCommandGroup(
                new SetArmAngle(() -> 0.0, a, true), 
                new SetWristAngle(() -> 0.0, w, true)
            )
        );
    }
    
}
