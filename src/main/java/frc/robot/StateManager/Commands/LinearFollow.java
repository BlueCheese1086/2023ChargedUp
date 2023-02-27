package frc.robot.StateManager.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Elevator.ElevatorSubsystem;
import frc.robot.StateManager.StateManager;

public class LinearFollow extends CommandBase {
    
    private final ElevatorSubsystem e;
    private final ArmSubsystem a;

    private final DoubleSupplier d;

    public LinearFollow(ElevatorSubsystem e, ArmSubsystem a, DoubleSupplier d) {
        this.d = d;
        this.a = a;
        this.e = e;
        addRequirements(a, e);
    }

    public void execute() {
        double[] ua = StateManager.getInstance().inverseIt(d.getAsDouble()*2.5, 1);
        e.setDesiredHeight(ua[0]);
        a.setAngle(ua[1]);
    }

}
