package frc.robot.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    public AbsoluteEncoder absoluteEncoder;

    public SparkMaxPIDController controller;

    public ArmState currentState;

    public ArmSubsystem(boolean inStartingPos) {
        arm = new CANSparkMax(ArmConstants.armId, MotorType.kBrushless);

        arm.restoreFactoryDefaults();

        arm.setIdleMode(IdleMode.kCoast);

        neoEncoder = arm.getEncoder();
        absoluteEncoder = arm.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(2*Math.PI/*/ArmConstants.GEARBOX_RATIO*/);
        absoluteEncoder.setZeroOffset(5.24);

        // neoEncoder.setPositionConversionFactor(2*Math.PI / ArmConstants.GEARBOX_RATIO);
        // neoEncoder.setPosition((absoluteEncoder.getPosition()/*-absoluteEncoder.getZeroOffset()*/)*(2*Math.PI / ArmConstants.GEARBOX_RATIO));

        controller = arm.getPIDController();

        controller.setP(ArmConstants.kP);
        controller.setI(ArmConstants.kI);
        controller.setD(ArmConstants.kD);
        controller.setFF(ArmConstants.kFF);
        controller.setFeedbackDevice(absoluteEncoder);

        currentState = new ArmState(0, 0, 0);
		Shuffleboard.getTab("Debug").add("PID", debugController);
    }
    
    PIDController debugController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
	PIDController lastController = debugController;
	@Override
	public void periodic() {
		if (!debugController.equals(lastController)) {
			controller.setP(debugController.getP());
			controller.setI(debugController.getI());
			controller.setD(debugController.getD());
		}
        lastController = debugController;
        // System.out.println(neoEncoder.getPosition());
        // controller.setReference(0, null)
        currentState = new ArmState(absoluteEncoder.getPosition(), neoEncoder.getVelocity(), absoluteEncoder.getPosition());//-absoluteEncoder.getZeroOffset());
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
