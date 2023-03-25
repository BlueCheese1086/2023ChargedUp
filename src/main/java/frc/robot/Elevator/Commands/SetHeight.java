package frc.robot.Elevator.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Elevator.ElevatorSubsystem;

public class SetHeight extends CommandBase {

    private final ElevatorSubsystem elevator;

    private final DoubleSupplier doubleSup;

    public SetHeight(DoubleSupplier d, ElevatorSubsystem e) {
        doubleSup = d;
        elevator = e;
        addRequirements(e);
    }

    @Override
    public void execute() {
        elevator.setDesiredHeight(doubleSup.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getCurrentState().height - doubleSup.getAsDouble()) < ElevatorConstants.TOLERANCE;
    }
    
}
