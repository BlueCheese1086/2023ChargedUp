package frc.robot.Wrist.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.StateManager.StateManager;
import frc.robot.Wrist.WristSubsystem;

public class SetWristAngle extends CommandBase {

    private final WristSubsystem wrist;

    private final DoubleSupplier doubleSup;

    private final boolean fourbar;

    public SetWristAngle(DoubleSupplier d, WristSubsystem w, boolean fb) {
        doubleSup = d;
        wrist = w;
        fourbar = fb;
        addRequirements(w);
    }

    @Override
    public void execute() {
        wrist.setAngle(doubleSup.getAsDouble(), fourbar);
    }

    public boolean isFinished() {
        return Math.abs(wrist.getCurrentState().angle - doubleSup.getAsDouble() - (fourbar ? StateManager.getInstance().getArmState().angle : 0.0)) < WristConstants.TOLERANCE;
    }

    public void end(boolean interr) {}
    
}
