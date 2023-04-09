package frc.robot.StateManager.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.StateManager.StateManager;
import frc.robot.StateManager.StateManager.Positions;

public class AutoSetPosition extends CommandBase {

    private final Positions p;
    private final StateManager s;

    public AutoSetPosition(Positions p) {
        this.p = p;
        s = StateManager.getInstance();
        addRequirements(s);
    }

    @Override
    public void execute() {
        s.setPosition(p);
        s.executePosition();
    }

    public boolean isFinished() {
        return s.armAtSetpoint() && s.wristAtSetpoint();
    }
    
}
