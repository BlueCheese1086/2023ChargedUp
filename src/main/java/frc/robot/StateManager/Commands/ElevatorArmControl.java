package frc.robot.StateManager.Commands;

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

    public ElevatorArmControl(StateManager v, ArmSubsystem a, ElevatorSubsystem e, WristSubsystem w) {
        this.a = a;
        this.e = e;
        this.w = w;
        this.v = v;
        addRequirements(v, a, e, w);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Positions current = v.getCurrentPosition();
        Positions desired = v.getDesiredPosition();

        double[] vals = current.getEffectors();
        e.setDesiredHeight(vals[0]);
        a.setAngle(vals[1]);
        if (vals[2] != -100) {
            w.setAngle(vals[2]);
        }

        if (current != desired && current.isHere(vals)) {
            v.setPosition(desired);
        }

    }

}
