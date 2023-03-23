package frc.robot.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Auto.SystemsCheck.SubChecker;
import frc.robot.Auto.SystemsCheck.SystemsCheck;
import frc.robot.Constants.ArmConstants;
import frc.robot.Util.DebugPID;

public class ArmSubsystem extends SubsystemBase implements SubChecker {

    public CANSparkMax arm;

    public RelativeEncoder relEncoder;
    public SparkMaxAbsoluteEncoder absoluteEncoder;

    public SparkMaxPIDController controller;

    public ArmState state;

    public ArmSubsystem() {
        arm = new CANSparkMax(ArmConstants.armId, MotorType.kBrushless);

        arm.restoreFactoryDefaults();

        arm.setIdleMode(IdleMode.kCoast);

        arm.setInverted(false);

        relEncoder = arm.getEncoder();
        absoluteEncoder = arm.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        absoluteEncoder.setZeroOffset(ArmConstants.ENC_OFFSET);

        relEncoder.setPositionConversionFactor(2*Math.PI / ArmConstants.GEARBOX_RATIO);

        if (absoluteEncoder.getPosition() > ArmConstants.UPPER_RANGE) {
            relEncoder.setPosition(2.0*Math.PI - absoluteEncoder.getPosition());
        } else {
            relEncoder.setPosition(absoluteEncoder.getPosition());
        }

        controller = arm.getPIDController();

        // controller.setFeedbackDevice(absoluteEncoder);

        controller.setP(ArmConstants.kP);
        controller.setI(ArmConstants.kI);
        controller.setD(ArmConstants.kD);
        controller.setFF(ArmConstants.kFF);

        state = new ArmState(1, 0);

        Shuffleboard.getTab("Arm").addNumber("Absolute Encoder POS", () -> absoluteEncoder.getPosition());

        new DebugPID(controller, "ARM");
    }
    
    @Override
    public void periodic() {
        state = new ArmState(relEncoder.getPosition(), relEncoder.getVelocity());
    }

    public ArmState getCurrentState() {
        return this.state;
    }

    public void setAngle(double a) {
        if (Robot.isSimulation()) {
            relEncoder.setPosition(a);
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
            if (relEncoder.getPosition() > 0.05) {
                works = true;
            }
            SystemsCheck.setSystemStatus(this, works);
            reset();
            }, this);
    }

}
