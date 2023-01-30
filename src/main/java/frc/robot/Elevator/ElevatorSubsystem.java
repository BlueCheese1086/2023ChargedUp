package frc.robot.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    
    private final CANSparkMax left;
    private final CANSparkMax right;

    private final RelativeEncoder leftEncoder;
    private final SparkMaxPIDController leftPID;

    private final DigitalInput bottomSwitch;

    private ElevatorState currentState = new ElevatorState(0, 0);

    public enum ElevatorPosition {
        Stowed,
        Low,
        Mid,
        High,
        PlayerStation;
    }

    public ElevatorSubsystem(int leftID, int rightID, int limitSwitchID, boolean inStaringPos) {
        left = new CANSparkMax(leftID, MotorType.kBrushless);
        right = new CANSparkMax(rightID, MotorType.kBrushless);

        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();

        right.setInverted(true);
        right.follow(left);

        leftEncoder = left.getEncoder();
        //Motor position to elevator height (meters)
        leftEncoder.setPositionConversionFactor(ElevatorConstants.SPOOL_RADIUS*2*Math.PI/ElevatorConstants.GEARBOX_RATIO);
        //Motor velocity m/s
        leftEncoder.setVelocityConversionFactor(ElevatorConstants.SPOOL_RADIUS*2*Math.PI/ElevatorConstants.GEARBOX_RATIO/60);

        leftPID = left.getPIDController();
        leftPID.setP(ElevatorConstants.kP);
        leftPID.setI(ElevatorConstants.kI);
        leftPID.setD(ElevatorConstants.kD);
        leftPID.setFF(ElevatorConstants.kFF);


        bottomSwitch = new DigitalInput(limitSwitchID);

        if (inStaringPos && bottomSwitch.get()) {
            leftEncoder.setPosition(-0.05);
        }
    }

    @Override
    public void periodic() {
        currentState = new ElevatorState(leftEncoder.getPosition(), leftEncoder.getVelocity());
    }

    /**
     * Set the desired elevator height
     * @param height Height in meters
     */
    public void setDesiredHeight(double height) {
        leftPID.setReference(height, ControlType.kPosition);
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


}
