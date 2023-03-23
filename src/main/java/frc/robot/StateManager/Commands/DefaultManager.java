package frc.robot.StateManager.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateManager.StateManager;

public class DefaultManager extends CommandBase {

    private final StateManager state;
    
    public DefaultManager() {
        state = StateManager.getInstance();
        addRequirements(state);
    }

    @Override
    public void execute() {
        state.executePosition();
    }
    
}
