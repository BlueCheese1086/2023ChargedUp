package frc.robot.StateManager.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Elevator.ElevatorSubsystem;
import frc.robot.StateManager.StateManager;
import frc.robot.Wrist.WristSubsystem;

public class DefaultManager extends CommandBase {

    private final StateManager state;
    
    public DefaultManager(ArmSubsystem a, WristSubsystem w, ElevatorSubsystem e) {
        state = StateManager.getInstance();
        addRequirements(state, a, w);
    }

    @Override
    public void execute() {
        state.executePosition();
    }
    
}
