package frc.robot.Elevator.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Elevator.ElevatorSubsystem;

public class RawControl extends CommandBase {
    
    private final ElevatorSubsystem e;
    private final DoubleSupplier d;

    double elapsed = System.currentTimeMillis();

    public RawControl(ElevatorSubsystem e, DoubleSupplier d) {
        this.e = e;
        this.d = d;
        addRequirements(e);
    }

    @Override
    public void execute() {
        //e.setDesiredHeight(e.getCurrentState().height+(.1*d.getAsDouble()));
        e.set(d.getAsDouble()*0.1);
    }


}
