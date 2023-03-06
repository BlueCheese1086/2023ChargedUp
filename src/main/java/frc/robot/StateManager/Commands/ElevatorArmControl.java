package frc.robot.StateManager.Commands;

import java.util.function.IntSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Elevator.ElevatorSubsystem;
import frc.robot.StateManager.StateManager;
import frc.robot.StateManager.StateManager.Positions;
import frc.robot.Wrist.WristSubsystem;

public class ElevatorArmControl extends CommandBase {

    private final ArmSubsystem a;
    private final ElevatorSubsystem e;
    private final WristSubsystem w;
    private final StateManager v;

    private final IntSupplier i;

    public ElevatorArmControl(StateManager v, ArmSubsystem a, ElevatorSubsystem e, WristSubsystem w, IntSupplier i) {
        this.a = a;
        this.e = e;
        this.w = w;
        this.v = v;
        this.i = i;
        addRequirements(v, a, e, w);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Positions current = v.getCurrentPosition();
        Positions desired = v.getDesiredPosition();

        double[] positions = new double[] {
                e.getCurrentState().height,
                a.getCurrentState().angle,
                w.getCurrentState().angle
        };

        double[] vals = current.getValue();
        // e.setDesiredHeight(vals[0]);
        a.setAngle(vals[1]);
        // w.setAngle(Units.degreesToRadians(72));
        // if (vals[2] != -100) {
        //     w.setAngle(vals[2]);
        // }

        // if (current != desired && current.isHere(vals)) {
        //     v.setPosition(desired);
        // }

    }

}
