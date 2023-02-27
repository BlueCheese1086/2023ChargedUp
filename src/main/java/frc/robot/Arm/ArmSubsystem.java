package frc.robot.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Auto.SystemsCheck.SubChecker;
import frc.robot.Auto.SystemsCheck.SystemsCheck;
import frc.robot.Constants.ArmConstants;

/*
 * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠛⠛⠻⢿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠏⠀⠀⠀⠀⠀⢹⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠋⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡟⠁⠀⠀⠀⠀⠀⠀⣰⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠏⠀⠀⠀⠀⠀⠀⢀⣴⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠏⠀⠀⠀⠀⠀⢀⣤⣾⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠏⠀⠀⠀⢀⣠⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠃⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠀⠀⠀⠀⢀⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠏⠀⠀⠀⣠⣶⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⣿⣿⡿⠃⠀⢀⣤⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⡿⠛⠁⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⠉⠀⠀⠀⢀⣀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣧⠖⢀⠄⢠⣾⣿⣀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣶⣿⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 */

public class ArmSubsystem extends SubsystemBase implements SubChecker {

    public CANSparkMax arm;

    public RelativeEncoder neoEncoder;
    public RelativeEncoder throughBoreEncoder;
    public AbsoluteEncoder absoluteEncoder;

    public SparkMaxPIDController controller;

    public ArmState currentState;

    public ArmSubsystem(boolean inStartingPos) {
        arm = new CANSparkMax(ArmConstants.armId, MotorType.kBrushless);

        arm.restoreFactoryDefaults();

        neoEncoder = arm.getEncoder();
        // throughBoreEncoder = arm.getAlternateEncoder(8192);
        absoluteEncoder = arm.getAbsoluteEncoder(com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setZeroOffset(ArmConstants.ENC_OFFSET);

        neoEncoder.setPositionConversionFactor(2*Math.PI / ArmConstants.GEARBOX_RATIO);
        if (inStartingPos) neoEncoder.setPosition(absoluteEncoder.getPosition()-absoluteEncoder.getZeroOffset());

        controller = arm.getPIDController();

        controller.setP(ArmConstants.kP);
        controller.setI(ArmConstants.kI);
        controller.setD(ArmConstants.kD);
        controller.setFF(ArmConstants.kFF);

        currentState = new ArmState(0, 0);
    }
    
    @Override
    public void periodic() {
        currentState = new ArmState(neoEncoder.getPosition(), neoEncoder.getVelocity());
    }

    public ArmState getCurrentState() {
        return this.currentState;
    }

    public void setAngle(double a) {
        if (Robot.isSimulation()) {
            neoEncoder.setPosition(a);
            return;
        }
        controller.setReference(a, ControlType.kPosition);
    }

    public void reset() {
        setAngle(0);
    } 

    public Command check(boolean safe) {
        if (!safe) return new InstantCommand();
        return new InstantCommand(() -> {
            double start = System.currentTimeMillis();
            boolean works = false;
            setAngle(0.2);
            while (System.currentTimeMillis() - start < 2000) {}
            if (neoEncoder.getPosition() > 0.05) {
                works = true;
            }
            SystemsCheck.setSystemStatus(this, works);
            reset();
            }, this);
    }

}
