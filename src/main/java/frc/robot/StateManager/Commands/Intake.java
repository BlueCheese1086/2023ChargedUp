package frc.robot.StateManager.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.StateManager.StateManager;
import frc.robot.Wrist.WristSubsystem;

public class Intake extends CommandBase {

    private final boolean dip;

    public Intake(ArmSubsystem a, WristSubsystem w, IntakeSubsystem i, boolean dip){
        this.dip = dip;
        addRequirements(a, w, i, StateManager.getInstance());
    }

    @Override
    public void execute() {
        StateManager.getInstance().intake(dip);
    }
    
}
