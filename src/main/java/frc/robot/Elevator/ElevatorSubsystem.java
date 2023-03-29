package frc.robot.Elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.ExternalFollower;
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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Util.DebugPID;

public class ElevatorSubsystem extends SubsystemBase implements SubChecker {
    
    private final CANSparkMax left;
    private final CANSparkMax right;

    private final RelativeEncoder relEncoder;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkMaxPIDController controller;


    private ElevatorState currentState = new ElevatorState(0, 0);

    public enum ElevatorPosition {
        Stowed,
        Low,
        Mid,
        High,
        PlayerStation;
    }

    public ElevatorSubsystem() {
        left = new CANSparkMax(ElevatorConstants.leftID, MotorType.kBrushless);
        right = new CANSparkMax(ElevatorConstants.rightID, MotorType.kBrushless);

        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();

        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);

        left.setInverted(false);
        right.setInverted(true);

        left.follow(ExternalFollower.kFollowerDisabled, 0);
        left.follow(right, true);

        // right.setSmartCurrentLimit(15);

        relEncoder = right.getEncoder();
        absoluteEncoder = left.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setInverted(true);
        absoluteEncoder.setPositionConversionFactor(ElevatorConstants.SPOOL_DIAMETER * Math.PI);
        absoluteEncoder.setZeroOffset(ElevatorConstants.ABS_OFFSET);

        relEncoder.setPositionConversionFactor(ElevatorConstants.SPOOL_DIAMETER*Math.PI/ElevatorConstants.GEARBOX_RATIO);
        relEncoder.setVelocityConversionFactor(ElevatorConstants.SPOOL_DIAMETER*Math.PI/ElevatorConstants.GEARBOX_RATIO/60);
        relEncoder.setPosition(absoluteEncoder.getPosition());

        controller = right.getPIDController();
        controller.setP(ElevatorConstants.kP);
        controller.setI(ElevatorConstants.kI);
        controller.setD(ElevatorConstants.kD);
        controller.setFF(ElevatorConstants.kFF);

        controller.setFeedbackDevice(relEncoder);

        // if (inStaringPos && bottomSwitch.get()) {
        //     leftEncoder.setPosition(-0.05);
        // }
        new DebugPID(controller, "Elevator");
        Shuffleboard.getTab("Subsystems").addBoolean("Elevator", () -> true);
        Shuffleboard.getTab("Elevator").addNumber("ABS", () -> absoluteEncoder.getPosition());
    }

    @Override
    public void periodic() {
        currentState = new ElevatorState(relEncoder.getPosition(), relEncoder.getVelocity());
    }

    /**
     * Set the desired elevator height
     * @param height Height in meters
     */
    public void setDesiredHeight(double height) {
        if (Robot.isSimulation()) {
            relEncoder.setPosition(height);
            return;
        }
        // if (Math.abs(leftEncoder.getVelocity()) > 0.05) {
        //     leftPID.setReference(Math.signum(leftEncoder.getVelocity()) * 0.05, ControlType.kVelocity);
        //     return;
        // }
        controller.setReference(height, ControlType.kPosition);
    }

    /**
     * Set the desired preset position
     * @param e The desired position
     */
    public void setDesiredPosition(ElevatorPosition e) {
        switch (e) {
            case Stowed:
                setDesiredHeight(0);
                break;
            case Low:
                setDesiredHeight(0);
                break;
            case Mid:
                setDesiredHeight(0);
                break;
            case High:
                setDesiredHeight(0);
                break;
            case PlayerStation:
                setDesiredHeight(0);
                break;
        }
    }

    /**
     * Gets the current state of the elevator
     * @return The current state of the elevator
     */
    public ElevatorState getCurrentState() {
        return this.currentState;
    }

    public Command check(boolean safe) {
        if (!safe) {
          return new InstantCommand();
        }
        return new InstantCommand(() -> {
          double time = System.currentTimeMillis();
          boolean working = false;
          setDesiredHeight(0.1);
          while (System.currentTimeMillis() - time < 2000) {}
          if (relEncoder.getPosition() > 0.05) {
            working = true;
          }
          SystemsCheck.setSystemStatus(this, working);
        }, this);
      }


}
