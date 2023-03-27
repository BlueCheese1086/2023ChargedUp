package frc.robot.Intake.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.StateManager.StateManager;

public class DefaultIntake extends CommandBase {

    private final IntakeSubsystem intake;

    private final DoubleSupplier supplier;

    public DefaultIntake(IntakeSubsystem i, DoubleSupplier s) {
        intake = i;
        supplier = s;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intake(supplier.getAsDouble());
        // intake.intake(Math.signum(StateManager.getInstance().getPieceMode().getIntakeSpeed())*supplier.getAsDouble());
    }

    @Override
    public void end(boolean interr) {
        intake.intake(0.0);
    }
    
}
