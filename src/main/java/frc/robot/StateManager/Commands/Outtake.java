package frc.robot.StateManager.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.StateManager.StateManager;
import frc.robot.Wrist.WristSubsystem;

public class Outtake extends CommandBase {

    public Outtake(ArmSubsystem a, WristSubsystem w, IntakeSubsystem i){
        addRequirements(a, w, i, StateManager.getInstance());
    }

    @Override
    public void execute() {
        StateManager.getInstance().score();
    }
    
}
