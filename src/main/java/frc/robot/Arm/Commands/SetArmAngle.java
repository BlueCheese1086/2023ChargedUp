package frc.robot.Arm.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Constants.ArmConstants;

public class SetArmAngle extends CommandBase {

    private final ArmSubsystem arm;

    private final DoubleSupplier doubleSup;

    private final boolean ignoreLimit;

    public SetArmAngle(DoubleSupplier d, ArmSubsystem a, boolean ignoreLimit) {
        doubleSup = d;
        arm = a;
        this.ignoreLimit = ignoreLimit;
        addRequirements(a);
    }

    @Override
    public void execute() {
        if (ignoreLimit) {
            arm.setAngleIgnoreLimit(doubleSup.getAsDouble());
        }
        else {
            arm.setAngle(doubleSup.getAsDouble());
        }
    }

    public boolean isFinished() {
        return Math.abs(arm.getCurrentState().angle - doubleSup.getAsDouble()) < ArmConstants.TOLERANCE;
    }

    public void end(boolean interr) {}
    
}
