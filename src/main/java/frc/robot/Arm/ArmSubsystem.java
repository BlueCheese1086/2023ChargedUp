package frc.robot.Arm;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Configuration.ControllableConfiguration;
import frc.robot.Configuration.DebugPID;
import frc.robot.Constants.ArmConstants;
import frc.robot.SparkMaxUtils.SparkMax;

public class ArmSubsystem extends SubsystemBase {

    public SparkMax arm;

    public RelativeEncoder relEncoder;
    public SparkMaxAbsoluteEncoder absoluteEncoder;

    public SparkMaxPIDController controller;

    public ArmState state;

    private final HashMap<String, ControllableConfiguration> configurations = new HashMap<>();

    private double referencePoint = 0.0;

    public ArmSubsystem() {
        arm = new SparkMax("Arm Motor", ArmConstants.armId, MotorType.kBrushless);

        arm.restoreFactoryDefaults();
        arm.setIdleMode(IdleMode.kBrake);
        arm.setInverted(false);
        arm.setSmartCurrentLimit(25);

        relEncoder = arm.getEncoder();
        absoluteEncoder = arm.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        absoluteEncoder.setZeroOffset(ArmConstants.ENC_OFFSET);

        controller = arm.getPIDController();
        controller.setFeedbackDevice(absoluteEncoder);
        controller.setP(ArmConstants.kP);
        controller.setI(ArmConstants.kI);
        controller.setD(ArmConstants.kD);
        controller.setFF(ArmConstants.kFF);

        state = new ArmState(1, 0);

        Shuffleboard.getTab("Arm").addNumber("Absolute Encoder POS", () -> absoluteEncoder.getPosition());

        configurations.putAll(Map.ofEntries(
            Map.entry("Enabled", new ControllableConfiguration("Subsystems", "Arm Enabled", true))
        ));

        new DebugPID(controller, "ARM");
    }
    
    @Override
    public void periodic() {
        if (!(Boolean) configurations.get("Enabled").getValue() || (arm.getFault(FaultID.kStall) && arm.getOutputCurrent() > 1)) {
            arm.stopMotor();
        }
        state = new ArmState(absoluteEncoder.getPosition() - Math.PI, absoluteEncoder.getVelocity());
    }

    public ArmState getCurrentState() {
        return this.state;
    }

    public void setAngle(double a) {
        if (Robot.isSimulation()) {
            relEncoder.setPosition(a);
            return;
        }
        referencePoint = a + Math.PI;
        if (a > ArmConstants.UPPER_RANGE) {
            referencePoint = ArmConstants.UPPER_RANGE + Math.PI;
            return;
        }
        if (a < ArmConstants.LOWER_RANGE) {
            referencePoint = ArmConstants.LOWER_RANGE + Math.PI;
            return;
        }
        controller.setReference(referencePoint, ControlType.kPosition);
    }

    public boolean atSetpoint(double t) {
        return Math.abs(referencePoint - Math.PI - this.getCurrentState().angle) < t;
    }

    public void setAngleIgnoreLimit(double a) {
        if (Robot.isSimulation()) {
            relEncoder.setPosition(a);
            return;
        }
        controller.setReference(a + Math.PI, ControlType.kPosition);
    }

    public void reset() {
        setAngle(0);
    }

}